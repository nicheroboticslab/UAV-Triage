#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    BatteryStatus,
    EstimatorStatusFlags,
)
import time
import subprocess
import re
import json
import math
import threading
import os
from datetime import datetime
from pynput import keyboard
from enum import Enum


# ============================================================
#  INTENT DEFINITIONS
# ============================================================
class Intent(Enum):
    HOLD_SAFE               = 1
    CONTINUE_ASSIGNED_TASK  = 2
    SEARCH_NEXT_PRIORITY    = 3
    INSPECT_TARGET_REPORT   = 4
    COMMS_REACQUIRE         = 5
    RETURN_TO_BASE          = 6
    ABORT_AND_SAFE          = 7


# ============================================================
#  MAP DEFINITION
# ============================================================
MAP = {
    "sectors": [
        {"id": 0, "name": "SECTOR_A", "priority": 1, "pos": [100.0,   0.0, -10.0]},
        {"id": 1, "name": "SECTOR_B", "priority": 2, "pos": [ 50.0, 100.0, -10.0]},
        {"id": 2, "name": "SECTOR_C", "priority": 3, "pos": [  0.0,-100.0, -10.0]},
        {"id": 3, "name": "SECTOR_D", "priority": 4, "pos": [-100.0,  0.0, -10.0]},
        {"id": 4, "name": "SECTOR_E", "priority": 5, "pos": [ 80.0,  80.0, -10.0]},
    ],
    "base":        [0.0,  0.0,  -5.0],
    "comms_point": [30.0, 30.0, -25.0],
    "targets": [
        {"id": 0, "pos": [ 60.0,  40.0, -8.0], "confirmed": False},
        {"id": 1, "pos": [-40.0,  60.0, -8.0], "confirmed": False},
    ],
}

# ============================================================
#  THRESHOLDS & TIMINGS
# ============================================================
KEYBOARD_TIMEOUT             = 15.0
BATTERY_LOW_THRESH           = 25.0
BATTERY_CRIT_THRESH          = 15.0
EKF_CONF_MIN                 = 0.4
COMMLOSS_SCORE_THRESH        = 1
WAYPOINT_REACH_DIST          = 3.0
DWELL_TIME                   = 20.0
INTENT_REQUERY_INTERVAL      = 30.0
EVENT_LOG_MAX_ENTRIES        = 30
CONFIDENCE_HIGH              = 70
CONFIDENCE_LOW               = 50
SIGNAL_LOSS_PATTERN_THRESHOLD = 2
LOITER_RADIUS                = 15.0   # metres
LOITER_OMEGA                 = 0.3    # rad/s  (~21s per orbit)

DASHBOARD_STATE_FILE  = "/tmp/uav_dashboard_state.json"
DASHBOARD_LOG_DISPLAY = 8
DATASET_DIR           = os.path.expanduser("~/uav_dataset")


# ============================================================
#  DRONE STATES
# ============================================================
class DroneState(Enum):
    MANUAL        = "MANUAL"
    AUTO_FLYING   = "AUTO_FLYING"
    AUTO_DWELLING = "AUTO_DWELLING"
    AUTO_INIT     = "AUTO_INIT"


# ============================================================
#  DATASET COLLECTOR
# ============================================================
class DatasetCollector:
    def __init__(self, flight_start_time: float):
        os.makedirs(DATASET_DIR, exist_ok=True)
        sid             = datetime.fromtimestamp(flight_start_time).strftime("%Y%m%d_%H%M%S")
        self.filepath   = os.path.join(DATASET_DIR, f"flight_{sid}.jsonl")
        self.session_id = sid
        self.lock       = threading.Lock()
        self.records    = []
        self.decision_count   = 0
        self.last_decision_id = None
        print(f"[DATASET] Recording to: {self.filepath}")

    def _make_id(self) -> str:
        self.decision_count += 1
        return f"flight_{self.session_id}_{self.decision_count:03d}"

    def record_decision(self, **kw) -> str:
        rid = self._make_id()
        self.last_decision_id = rid
        record = {
            "id":          rid,
            "timestamp":   time.time(),
            "flight_time": kw.get("flight_time_str", ""),
            "input": {
                "battery_pct":              round(kw.get("battery_pct", 0), 1),
                "ekf_confidence":           round(kw.get("ekf_confidence", 0), 2),
                "drone_state":              kw.get("drone_state", ""),
                "current_wp":               kw.get("current_wp", ""),
                "current_pos":              [round(v, 1) for v in kw.get("current_pos", [0,0,0])],
                "signal_loss_count":        kw.get("signal_loss_count", 0),
                "post_signal_loss_reentry": kw.get("post_reentry", False),
                "sectors_visited":          kw.get("sectors_visited", []),
                "sectors_remaining":        kw.get("sectors_remaining", []),
                "event_log_at_decision":    kw.get("event_log_snapshot", [])[-15:],
                "last_loss_duration_s":     round(kw.get("last_loss_duration", 0), 1),
                "last_loss_battery_cost":   round(kw.get("last_loss_battery_cost", 0), 1),
                "last_loss_drift_m":        round(kw.get("last_loss_drift", 0), 1),
                "pre_loss_intent":          kw.get("pre_loss_intent", "None"),
                "pre_loss_wp":              kw.get("pre_loss_wp", "None"),
                "battery_low_thresh":       BATTERY_LOW_THRESH,
                "battery_crit_thresh":      BATTERY_CRIT_THRESH,
                "ekf_conf_min":             EKF_CONF_MIN,
            },
            "llm_output": {
                "intent":                kw.get("intent", Intent.HOLD_SAFE).value,
                "intent_name":           kw.get("intent", Intent.HOLD_SAFE).name,
                "target_wp":             kw.get("wp_name", ""),
                "reasoning":             kw.get("reasoning", ""),
                "confidence":            kw.get("confidence", 0),
                "next_checkpoint":       kw.get("checkpoint", ""),
                "response_time_seconds": round(kw.get("response_time_seconds", 0), 2),
                "fallback_used":         kw.get("fallback_used", False),
                "was_reentry_decision":  kw.get("was_reentry", False),
            },
            "outcome": {
                "waypoint_reached":             None,
                "battery_at_arrival":           None,
                "ekf_at_arrival":               None,
                "time_to_arrive_seconds":       None,
                "another_signal_loss_occurred": None,
                "dwell_completed":              None,
            },
            "label": {
                "correct":           None,
                "correct_intent":    None,
                "correct_reasoning": None,
                "notes":             None,
                "labeled_by":        None,
                "labeled_at":        None,
            }
        }
        with self.lock:
            self.records.append(record)
        threading.Thread(target=self._append, args=(record,), daemon=True).start()
        return rid

    def update_outcome(self, rid: str, **kw):
        with self.lock:
            for r in self.records:
                if r["id"] == rid:
                    r["outcome"].update({
                        "waypoint_reached":             kw.get("waypoint_reached"),
                        "battery_at_arrival":           round(kw.get("battery_at_arrival", 0), 1),
                        "ekf_at_arrival":               round(kw.get("ekf_at_arrival", 0), 2),
                        "time_to_arrive_seconds":       round(kw.get("time_to_arrive", 0), 1),
                        "another_signal_loss_occurred": kw.get("another_signal_loss", False),
                        "dwell_completed":              kw.get("dwell_completed", False),
                    })
                    break
        threading.Thread(target=self._rewrite, daemon=True).start()

    def _append(self, record):
        try:
            with open(self.filepath, 'a') as f:
                f.write(json.dumps(record) + "\n")
        except Exception as e:
            print(f"[DATASET] Append error: {e}")

    def _rewrite(self):
        try:
            tmp = self.filepath + ".tmp"
            with self.lock:
                snap = list(self.records)
            with open(tmp, 'w') as f:
                for r in snap:
                    f.write(json.dumps(r) + "\n")
            os.replace(tmp, self.filepath)
        except Exception as e:
            print(f"[DATASET] Rewrite error: {e}")

    def get_stats(self) -> dict:
        with self.lock:
            total     = len(self.records)
            fallbacks = sum(1 for r in self.records if r["llm_output"]["fallback_used"])
            reentries = sum(1 for r in self.records if r["llm_output"]["was_reentry_decision"])
            arrived   = sum(1 for r in self.records if r["outcome"]["waypoint_reached"] is True)
        return {"total_decisions": total, "fallback_count": fallbacks,
                "reentry_count": reentries, "arrived_count": arrived,
                "filepath": self.filepath}


# ============================================================
#  MAIN NODE
# ============================================================
class UAVAutonomousNode(Node):

    def __init__(self):
        super().__init__('uav_autonomous_node')

        qos_v = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_t = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           history=HistoryPolicy.KEEP_LAST, depth=5)

        # Publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub     = self.create_publisher(TrajectorySetpoint,  '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub      = self.create_publisher(VehicleCommand,      '/fmu/in/vehicle_command', 10)

        # Subscribers
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.pos_cb,     qos_v)
        self.create_subscription(BatteryStatus,        '/fmu/out/battery_status_v1',         self.battery_cb, qos_t)
        self.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags',    self.ekf_cb,     qos_v)

        # Position
        self.current_pos     = [0.0, 0.0, -5.0]
        self.target_pos      = [0.0, 0.0, -5.0]
        self.pos_initialized = False

        # Battery
        self.battery_pct  = 100.0
        self.battery_volt = 0.0
        self.battery_amp  = 0.0

        # EKF
        self.ekf_confidence = 1.0

        # Keyboard
        self.last_key_time = time.time()

        # State machine
        self.drone_state     = DroneState.MANUAL
        self.current_intent  = None
        self.current_wp_name = "None"

        # LLM
        self.llm_running          = False
        self.last_llm_time        = 0.0
        self.last_llm_reasoning   = ""
        self.last_llm_confidence  = 0
        self.last_llm_checkpoint  = ""
        self.last_llm_intent_num  = 0
        self.last_llm_wp_name     = "None"
        self.llm_query_start_time = 0.0

        # Dwell
        self.dwell_start     = 0.0
        self.dwell_active    = False
        self.dwell_logged    = False
        self.dwell_remaining = 0.0

        # Loiter
        self.loitering     = False
        self.circle_angle  = 0.0
        self.circle_center = None

        # =====================================================
        #  FIX 1: visited_sectors SET — tracks all visited sectors
        #  Every sector id gets added here when drone arrives.
        #  When all 5 visited, set resets → full new sweep begins.
        # =====================================================
        self.visited_sectors = set()   # set of sector IDs already visited
        self.assigned_sector = 0       # used by CONTINUE_ASSIGNED_TASK

        # =====================================================
        #  FIX 2: Override flags for test keys T and Y
        #  When override=True, the ROS callback for that sensor
        #  simply re-applies the forced value instead of reading PX4.
        #  Override cleared automatically after LLM responds.
        # =====================================================
        self.ekf_override           = False
        self.ekf_override_value     = 0.2    # value held when override active
        self.battery_override       = False
        self.battery_override_value = 10.0   # value held when override active

        # Event log (Stage 1)
        self.flight_start_time = time.time()
        self.event_log         = []

        # Signal loss (Stage 3)
        self.signal_was_lost          = False
        self.signal_lost_time         = 0.0
        self.signal_lost_battery      = 0.0
        self.signal_lost_position     = None
        self.signal_loss_count        = 0
        self.post_signal_loss_reentry = False
        self.pre_loss_intent          = None
        self.pre_loss_wp_name         = "None"
        self.pre_loss_battery         = 0.0
        self.pre_loss_position        = None
        self.last_loss_duration       = 0.0
        self.last_loss_battery_cost   = 0.0
        self.last_loss_drift          = 0.0

        # Dataset (Stage 5)
        self.dataset                  = DatasetCollector(self.flight_start_time)
        self.current_record_id        = None
        self.waypoint_depart_time     = 0.0
        self.signal_loss_since_depart = False

        # Loop
        self.counter      = 0
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.loop)

        kbl = keyboard.Listener(on_press=self.on_key_press)
        kbl.daemon = True
        kbl.start()

        self.log_event(f"System started. Battery: {self.battery_pct:.0f}%. EKF: {self.ekf_confidence:.2f}")
        self._write_dashboard_state()
        self._print_startup_banner()

    # =========================================================
    #  EVENT LOG
    # =========================================================

    def log_event(self, msg: str):
        elapsed   = time.time() - self.flight_start_time
        mins      = int(elapsed // 60); secs = int(elapsed % 60)
        ts        = f"[T+{mins}m{secs:02d}s]" if mins > 0 else f"[T+{secs}s]"
        entry     = f"{ts} {msg}"
        self.event_log.append(entry)
        if len(self.event_log) > EVENT_LOG_MAX_ENTRIES:
            self.event_log.pop(0)
        self.get_logger().info(f"[LOG] {entry}")

    def get_event_log_string(self) -> str:
        return "\n".join(f"  {e}" for e in self.event_log) if self.event_log else "  No events yet."

    def _flight_time_str(self) -> str:
        e = time.time() - self.flight_start_time
        m = int(e // 60); s = int(e % 60)
        return f"T+{m}m{s:02d}s" if m > 0 else f"T+{s}s"

    # =========================================================
    #  DASHBOARD WRITER
    # =========================================================

    def _write_dashboard_state(self):
        e  = time.time() - self.flight_start_time
        m  = int(e // 60); s = int(e % 60)
        ft = f"T+{m}m{s:02d}s" if m > 0 else f"T+{s}s"
        dr = max(0.0, DWELL_TIME - (time.time() - self.dwell_start)) if self.dwell_active else 0.0
        c  = self.last_llm_confidence
        cl = "HIGH" if c >= CONFIDENCE_HIGH else ("MODERATE" if c >= CONFIDENCE_LOW else ("LOW" if c > 0 else "---"))
        ds = self.dataset.get_stats()
        vn = [s['name'] for s in MAP['sectors'] if s['id'] in self.visited_sectors]
        rn = [s['name'] for s in MAP['sectors'] if s['id'] not in self.visited_sectors]
        state = {
            "flight_time": ft, "timestamp": time.time(),
            "drone_state": self.drone_state.value, "llm_running": self.llm_running,
            "post_reentry": self.post_signal_loss_reentry,
            "dwell_active": self.dwell_active, "dwell_remaining": round(dr, 1),
            "battery_pct": round(self.battery_pct, 1),
            "battery_low": self.battery_pct < BATTERY_LOW_THRESH,
            "battery_crit": self.battery_pct < BATTERY_CRIT_THRESH,
            "ekf_confidence": round(self.ekf_confidence, 2),
            "ekf_ok": self.ekf_confidence >= EKF_CONF_MIN,
            "position": [round(v, 1) for v in self.current_pos],
            "signal_lost": self.signal_was_lost,
            "signal_loss_count": self.signal_loss_count,
            "last_loss_duration": round(self.last_loss_duration, 1),
            "last_loss_batt_cost": round(self.last_loss_battery_cost, 1),
            "last_loss_drift": round(self.last_loss_drift, 1),
            "current_wp": self.current_wp_name,
            "current_intent": self.current_intent.name if self.current_intent else "NONE",
            "llm_intent_num": self.last_llm_intent_num,
            "llm_intent_name": self.current_intent.name if self.current_intent else "NONE",
            "llm_wp_name": self.last_llm_wp_name,
            "llm_confidence": self.last_llm_confidence,
            "llm_conf_label": cl,
            "llm_reasoning": self.last_llm_reasoning,
            "llm_checkpoint": self.last_llm_checkpoint,
            "sectors_visited": vn,
            "sectors_remaining": rn,
            "event_log": self.event_log[-DASHBOARD_LOG_DISPLAY:],
            "dataset_decisions": ds["total_decisions"],
            "dataset_file": os.path.basename(ds["filepath"]),
            "ekf_override": self.ekf_override,
            "battery_override": self.battery_override,
        }
        try:
            tmp = DASHBOARD_STATE_FILE + ".tmp"
            with open(tmp, 'w') as f:
                json.dump(state, f, indent=2)
            os.replace(tmp, DASHBOARD_STATE_FILE)
        except Exception:
            pass

    # =========================================================
    #  SIGNAL LOSS CONTEXT
    # =========================================================

    def _build_signal_loss_context(self) -> str:
        if self.ekf_confidence >= 0.75:           ekf_r = f"{self.ekf_confidence:.2f} — GOOD"
        elif self.ekf_confidence >= EKF_CONF_MIN: ekf_r = f"{self.ekf_confidence:.2f} — MODERATE"
        else:                                      ekf_r = f"{self.ekf_confidence:.2f} — POOR"
        interrupted = (f"{self.pre_loss_intent.name} → {self.pre_loss_wp_name}"
                       if self.pre_loss_intent else "No active task")
        drift_note = "minimal" if self.last_loss_drift < 2 else ("moderate" if self.last_loss_drift < 5 else "SIGNIFICANT")
        batt_note  = "negligible" if self.last_loss_battery_cost < 2 else ("moderate" if self.last_loss_battery_cost < 5 else "HIGH")
        pattern    = (f"\n⚠️  PATTERN: Loss #{self.signal_loss_count} — consider routing away or RTB."
                      if self.signal_loss_count >= SIGNAL_LOSS_PATTERN_THRESHOLD else "")
        return f"""
=== ⚡ SIGNAL LOSS EVENT — ADDRESS THIS BEFORE DECIDING ===
  Lost for         : {self.last_loss_duration:.1f}s
  Battery at loss  : {self.pre_loss_battery:.1f}%  →  now: {self.battery_pct:.1f}%
  Battery cost     : {self.last_loss_battery_cost:.1f}% — {batt_note}
  Position drift   : {self.last_loss_drift:.1f}m — {drift_note}
  EKF at regain    : {ekf_r}
  Loss #           : {self.signal_loss_count} this flight
  Interrupted task : {interrupted}
{pattern}
Consider: Is it safe to resume? Is the drift acceptable? Is this a pattern?
============================================================"""

    # =========================================================
    #  TERMINAL DISPLAY
    # =========================================================

    def _print_startup_banner(self):
        log = self.get_logger().info
        log("╔══════════════════════════════════════════════════════════╗")
        log("║   UAV AUTONOMOUS SYSTEM — FINAL FIXED VERSION           ║")
        log("║                                                          ║")
        log("║  FIXES: All sectors visited | Test keys stable          ║")
        log("║                                                          ║")
        log("║  T = force EKF → 0.2   (LOCKED until you press R)       ║")
        log("║  Y = force Batt → 10%  (LOCKED until you press R)       ║")
        log("║  R = reset BOTH to normal                               ║")
        log("║  Z = simulate signal loss (5s)                          ║")
        log("║  F = force LLM re-query now                             ║")
        log("║  1-7 = force specific intent directly                   ║")
        log("║  H = show dataset stats                                 ║")
        log("╚══════════════════════════════════════════════════════════╝")

    def _print_pilot_decision_banner(self, intent, target_name, target_pos,
                                      confidence, reasoning, checkpoint,
                                      fallback_used=False, post_signal_loss=False):
        log = self.get_logger().info
        cl  = "✅ HIGH" if confidence >= CONFIDENCE_HIGH else ("⚠️  MOD" if confidence >= CONFIDENCE_LOW else "❌ LOW")
        log("╔══════════════════════════════════════════════════════════╗")
        h = "🛡️  FALLBACK" if fallback_used else ("📡 RE-ENTRY" if post_signal_loss else "🧠 PILOT BRAIN")
        log(f"║  {h} DECISION                                      ║")
        log(f"║  Intent    : [{intent.value}] {intent.name:<35}║")
        log(f"║  Target    : {target_name:<47}║")
        log(f"║  Confidence: {confidence}%  {cl:<40}║")
        log("║                                                          ║")
        words = reasoning.split(); line = ""
        for word in words:
            if len(line) + len(word) + 1 <= 54: line += ("" if not line else " ") + word
            else: log(f"║  {line:<56}║"); line = word
        if line: log(f"║  {line:<56}║")
        if checkpoint: log(f"║  Next: {checkpoint[:51]:<51}║")
        log("╚══════════════════════════════════════════════════════════╝")

    def _print_arrived_banner(self, wp_name):
        log = self.get_logger().info
        log("╔══════════════════════════════════════════════════════════╗")
        log(f"║  ✅ ARRIVED AT: {wp_name:<43}║")
        log(f"║  Hovering {DWELL_TIME:.0f}s then querying LLM                     ║")
        log("╚══════════════════════════════════════════════════════════╝")

    def _print_mode_change(self, mode, reason):
        log = self.get_logger().warn
        log(f"┌─────────────────────────────────────────────────────────┐")
        log(f"│  MODE → {mode:<49}│")
        log(f"│  {reason:<57}│")
        log(f"└─────────────────────────────────────────────────────────┘")

    def _print_signal_banner(self, event, details):
        log = self.get_logger().warn
        log(f"┌─────────────────────────────────────────────────────────┐")
        log(f"│  📡 {event:<53}│")
        log(f"│  {details:<57}│")
        log(f"└─────────────────────────────────────────────────────────┘")

    # =========================================================
    #  SUBSCRIBER CALLBACKS
    # =========================================================

    def pos_cb(self, msg):
        self.current_pos = [msg.x, msg.y, msg.z]
        if not self.pos_initialized:
            self.target_pos      = self.current_pos.copy()
            self.pos_initialized = True
            self.log_event(f"Position initialized: {[f'{v:.1f}' for v in self.current_pos]}")

    def battery_cb(self, msg):
        # =====================================================
        #  FIX 2: If battery override active, hold forced value
        #  and do NOT read from PX4 until override is cleared.
        # =====================================================
        if self.battery_override:
            self.battery_pct = self.battery_override_value
            return

        prev              = self.battery_pct
        self.battery_pct  = msg.remaining * 100.0
        self.battery_volt = msg.voltage_v
        self.battery_amp  = msg.current_a

        if prev >= BATTERY_LOW_THRESH > self.battery_pct:
            self.log_event(f"Battery LOW: {self.battery_pct:.1f}%")
        if prev >= BATTERY_CRIT_THRESH > self.battery_pct:
            self.log_event(f"Battery CRITICAL: {self.battery_pct:.1f}% — RTB required")
        if self.battery_pct < BATTERY_CRIT_THRESH and self.counter % 40 == 0:
            self.get_logger().error(f"[BATT] 🚨 CRITICAL {self.battery_pct:.1f}%")
        elif self.battery_pct < BATTERY_LOW_THRESH and self.counter % 200 == 0:
            self.get_logger().warn(f"[BATT] ⚠ LOW {self.battery_pct:.1f}%")

    def ekf_cb(self, msg):
        # =====================================================
        #  FIX 2: If EKF override active, hold forced value
        #  and do NOT compute from PX4 flags until cleared.
        # =====================================================
        if self.ekf_override:
            self.ekf_confidence = self.ekf_override_value
            return

        prev    = self.ekf_confidence
        penalty = 0.0
        if hasattr(msg, 'cs_tilt_align') and not msg.cs_tilt_align: penalty += 0.4
        if hasattr(msg, 'cs_yaw_align')  and not msg.cs_yaw_align:  penalty += 0.4
        if hasattr(msg, 'cs_gps')        and not msg.cs_gps:        penalty += 0.2
        if hasattr(msg, 'cs_baro_hgt')   and not msg.cs_baro_hgt:   penalty += 0.2
        if hasattr(msg, 'cs_opt_flow')   and not msg.cs_opt_flow:   penalty += 0.1
        if hasattr(msg, 'cs_mag_hdg')    and not msg.cs_mag_hdg:    penalty += 0.1
        self.ekf_confidence = max(0.0, min(1.0, 1.0 - penalty))

        if prev >= EKF_CONF_MIN > self.ekf_confidence:
            self.log_event(f"EKF dropped below min: {self.ekf_confidence:.2f}")
        if prev < EKF_CONF_MIN <= self.ekf_confidence:
            self.log_event(f"EKF recovered: {self.ekf_confidence:.2f}")

    # =========================================================
    #  KEYBOARD
    # =========================================================

    def on_key_press(self, key):
        step = 1.0
        try:
            moved = False
            if   key == keyboard.Key.up:    self.target_pos[0] += step; moved = True
            elif key == keyboard.Key.down:  self.target_pos[0] -= step; moved = True
            elif key == keyboard.Key.left:  self.target_pos[1] += step; moved = True
            elif key == keyboard.Key.right: self.target_pos[1] -= step; moved = True
            elif key.char == 'w': self.target_pos[2] -= step; moved = True
            elif key.char == 's': self.target_pos[2] += step; moved = True

            elif key.char == 'b':
                self.get_logger().info(
                    f"[BATT] {self.battery_pct:.1f}% | {self.battery_volt:.2f}V | {self.battery_amp:.2f}A"
                )
                return

            elif key.char == 'h':
                ds = self.dataset.get_stats()
                visited   = [s['name'] for s in MAP['sectors'] if s['id'] in self.visited_sectors]
                remaining = [s['name'] for s in MAP['sectors'] if s['id'] not in self.visited_sectors]
                self.get_logger().info(
                    f"[INFO] Dataset decisions={ds['total_decisions']} "
                    f"fallbacks={ds['fallback_count']} reentries={ds['reentry_count']}"
                )
                self.get_logger().info(f"[INFO] Visited={visited}  Remaining={remaining}")
                return

            elif key.char == 't':
                # =====================================================
                #  FIX 2: T = force EKF bad, SET override flag
                #  Override flag blocks ekf_cb from reverting the value
                #  Override will be cleared in _llm_worker after LLM responds
                # =====================================================
                self.ekf_override       = True
                self.ekf_override_value = 0.2
                self.ekf_confidence     = 0.2
                self.log_event("[TEST] EKF forced → 0.2 (LOCKED — press R to release)")
                self.get_logger().warn("[TEST] ⚠ EKF override ON → 0.2")
                self.last_llm_time = 0.0
                self.drone_state   = DroneState.AUTO_INIT
                return

            elif key.char == 'y':
                # =====================================================
                #  FIX 2: Y = force battery low, SET override flag
                # =====================================================
                self.battery_override       = True
                self.battery_override_value = 10.0
                self.battery_pct            = 10.0
                self.log_event("[TEST] Battery forced → 10% (LOCKED — press R to release)")
                self.get_logger().warn("[TEST] ⚠ Battery override ON → 10%")
                self.last_llm_time = 0.0
                self.drone_state   = DroneState.AUTO_INIT
                return

            elif key.char == 'r':
                # =====================================================
                #  FIX 2: R = reset BOTH overrides to normal
                # =====================================================
                self.ekf_override     = False
                self.battery_override = False
                self.ekf_confidence   = 1.0
                self.battery_pct      = 100.0
                self.log_event("[TEST] Reset: EKF → 1.0, Battery → 100% (overrides OFF)")
                self.get_logger().info("[TEST] ✅ Both overrides cleared — sensors reading normally")
                self.last_llm_time = 0.0
                self.drone_state   = DroneState.AUTO_INIT
                return

            elif key.char == 'f':
                self.last_llm_time = 0.0
                self.drone_state   = DroneState.AUTO_INIT
                self.dwell_active  = False
                self.log_event("Operator forced LLM re-query.")
                return

            elif key.char == 'z':
                def _sim():
                    self.pre_loss_intent      = self.current_intent
                    self.pre_loss_wp_name     = self.current_wp_name
                    self.pre_loss_battery     = self.battery_pct
                    self.pre_loss_position    = self.current_pos.copy()
                    self.signal_was_lost      = True
                    self.signal_lost_time     = time.time()
                    self.signal_lost_battery  = self.battery_pct
                    self.signal_lost_position = self.current_pos.copy()
                    self.signal_loss_count   += 1
                    self.signal_loss_since_depart = True
                    self.log_event(f"[TEST] Signal loss #{self.signal_loss_count} simulated (5s)")
                    time.sleep(5.0)
                    self.battery_pct             = max(0.0, self.battery_pct - 2.0)
                    duration                     = time.time() - self.signal_lost_time
                    self.last_loss_duration      = duration
                    self.last_loss_battery_cost  = self.signal_lost_battery - self.battery_pct
                    self.last_loss_drift         = 1.5
                    self.signal_was_lost         = False
                    self.post_signal_loss_reentry = True
                    self.log_event(
                        f"[TEST] Signal regained. Duration: {duration:.1f}s. "
                        f"Cost: {self.last_loss_battery_cost:.1f}%"
                    )
                    self._print_signal_banner("REGAINED (simulated)",
                        f"Duration: {duration:.1f}s | Cost: {self.last_loss_battery_cost:.1f}%")
                    self.last_llm_time = 0.0
                    self.drone_state   = DroneState.AUTO_INIT
                threading.Thread(target=_sim, daemon=True).start()
                return

            elif key.char in '1234567':
                intent = Intent(int(key.char))
                self.log_event(f"[TEST] Intent forced → {intent.name}")
                new_target, wp_name = self._generate_waypoint(intent)
                self.current_intent  = intent
                self.target_pos      = new_target
                self.current_wp_name = wp_name
                self.drone_state     = DroneState.AUTO_FLYING
                self.dwell_active    = False
                self._print_pilot_decision_banner(
                    intent, wp_name, new_target, 99,
                    f"Manually forced by operator (key {key.char}).",
                    "Until next operator input"
                )
                return

            if moved:
                self.last_key_time = time.time()
                self.loitering = False; self.circle_center = None
                self.dwell_active = False
                if self.drone_state != DroneState.MANUAL:
                    self.log_event(f"Human took control from {self.drone_state.value}. Batt: {self.battery_pct:.1f}%")
                    self.drone_state    = DroneState.MANUAL
                    self.current_intent = None
                    self._print_mode_change("MANUAL", "Human operator took control")

        except AttributeError:
            pass

    # =========================================================
    #  COMM LOSS / WAYPOINT
    # =========================================================

    def check_comm_loss(self) -> bool:
        score = 0
        if (time.time() - self.last_key_time) > KEYBOARD_TIMEOUT: score += 1
        if self.battery_pct < BATTERY_LOW_THRESH:                  score += 1
        if self.ekf_confidence < EKF_CONF_MIN:                     score += 1
        return score >= COMMLOSS_SCORE_THRESH

    def check_waypoint_reached(self) -> bool:
        if not self.pos_initialized: return False
        dx = self.current_pos[0] - self.target_pos[0]
        dy = self.current_pos[1] - self.target_pos[1]
        return math.sqrt(dx*dx + dy*dy) < WAYPOINT_REACH_DIST

    def _safe_fallback_intent(self) -> Intent:
        if self.battery_pct < BATTERY_CRIT_THRESH: return Intent.ABORT_AND_SAFE
        if self.ekf_confidence < EKF_CONF_MIN:      return Intent.HOLD_SAFE
        if self.battery_pct < BATTERY_LOW_THRESH:   return Intent.RETURN_TO_BASE
        return Intent.HOLD_SAFE

    # =========================================================
    #  LLM
    # =========================================================

    def trigger_llm(self):
        if self.llm_running: return

        # =====================================================
        #  HARD SAFETY GATE — runs BEFORE LLM is even queried
        #  If battery or EKF is in a critical state, skip the
        #  LLM entirely and apply deterministic safe action.
        #  This prevents the LLM from overriding safety rules.
        # =====================================================
        if self.battery_pct < BATTERY_CRIT_THRESH:
            intent = Intent.ABORT_AND_SAFE if self.battery_pct < 5.0 else Intent.RETURN_TO_BASE
            self.get_logger().error(
                f"[SAFETY] Battery CRITICAL ({self.battery_pct:.1f}%) — "
                f"forcing {intent.name}, skipping LLM"
            )
            self.log_event(f"Safety gate: battery {self.battery_pct:.1f}% → {intent.name} (no LLM)")
            new_target, wp_name  = self._generate_waypoint(intent)
            self.current_intent  = intent
            self.target_pos      = new_target
            self.current_wp_name = wp_name
            self.last_llm_time   = time.time()
            self.drone_state     = DroneState.AUTO_FLYING
            self.dwell_active    = False
            self._print_pilot_decision_banner(
                intent, wp_name, new_target, 99,
                f"Safety gate activated — battery critical at {self.battery_pct:.1f}%. "
                f"LLM bypassed. Returning home immediately.",
                "Land safely", fallback_used=True
            )
            return

        if self.ekf_confidence < EKF_CONF_MIN:
            self.get_logger().warn(
                f"[SAFETY] EKF critical ({self.ekf_confidence:.2f}) — "
                f"forcing HOLD_SAFE, skipping LLM"
            )
            self.log_event(f"Safety gate: EKF {self.ekf_confidence:.2f} → HOLD_SAFE (no LLM)")
            new_target, wp_name  = self._generate_waypoint(Intent.HOLD_SAFE)
            self.current_intent  = Intent.HOLD_SAFE
            self.target_pos      = new_target
            self.current_wp_name = wp_name
            self.last_llm_time   = time.time()
            self.drone_state     = DroneState.AUTO_FLYING
            self.dwell_active    = False
            self._print_pilot_decision_banner(
                Intent.HOLD_SAFE, wp_name, new_target, 99,
                f"Safety gate activated — EKF critical at {self.ekf_confidence:.2f}. "
                f"LLM bypassed. Holding safe position.",
                "Until EKF recovers", fallback_used=True
            )
            return
        # =====================================================

        self.llm_running          = True
        self.llm_query_start_time = time.time()
        tag = "📡 Re-entry..." if self.post_signal_loss_reentry else "🧠 Thinking..."
        self.get_logger().info(f"[LLM] {tag}")
        threading.Thread(target=self._llm_worker, daemon=True).start()

    def _llm_worker(self):
        was_reentry   = self.post_signal_loss_reentry
        fallback_used = False

        # Snapshot input state BEFORE query
        snap_batt       = self.battery_pct
        snap_ekf        = self.ekf_confidence
        snap_state      = self.drone_state.value
        snap_wp         = self.current_wp_name
        snap_pos        = self.current_pos.copy()
        snap_loss_count = self.signal_loss_count
        snap_visited    = [s['name'] for s in MAP['sectors'] if s['id'] in self.visited_sectors]
        snap_remaining  = [s['name'] for s in MAP['sectors'] if s['id'] not in self.visited_sectors]
        snap_log        = list(self.event_log)
        snap_loss_dur   = self.last_loss_duration
        snap_loss_cost  = self.last_loss_battery_cost
        snap_loss_drift = self.last_loss_drift
        snap_pre_intent = self.pre_loss_intent.name if self.pre_loss_intent else "None"
        snap_pre_wp     = self.pre_loss_wp_name

        try:
            intent, reasoning, confidence, checkpoint = self._query_llm_intent()
            response_time = time.time() - self.llm_query_start_time

            if confidence < CONFIDENCE_LOW:
                self.log_event(f"LLM conf {confidence}% too low → safe fallback (wanted {intent.name})")
                intent        = self._safe_fallback_intent()
                reasoning     = (f"LLM uncertain ({confidence}%). "
                                 f"Safe fallback: batt={snap_batt:.1f}%, EKF={snap_ekf:.2f}.")
                confidence    = 99
                checkpoint    = "Until conditions stabilize"
                fallback_used = True
            elif confidence < CONFIDENCE_HIGH:
                self.log_event(f"LLM moderate conf ({confidence}%): {intent.name}. {reasoning[:80]}")
            else:
                tag = " [RE-ENTRY]" if was_reentry else ""
                self.log_event(f"LLM{tag} ({confidence}%): {intent.name}. {reasoning[:80]}")

            self.last_llm_reasoning  = reasoning
            self.last_llm_confidence = confidence
            self.last_llm_checkpoint = checkpoint
            self.last_llm_intent_num = intent.value
            self.post_signal_loss_reentry = False

            # Overrides are NOT cleared here.
            # They stay locked until operator presses R.
            # This keeps the forced value stable across multiple LLM queries.

            new_target, wp_name  = self._generate_waypoint(intent)
            self.last_llm_wp_name = wp_name
            self.current_intent   = intent
            self.target_pos       = new_target
            self.current_wp_name  = wp_name
            self.last_llm_time    = time.time()
            self.dwell_active     = False
            self.dwell_logged     = False
            self.drone_state      = DroneState.AUTO_FLYING

            self.waypoint_depart_time     = time.time()
            self.signal_loss_since_depart = False

            # Record to dataset
            rid = self.dataset.record_decision(
                flight_time_str        = self._flight_time_str(),
                battery_pct            = snap_batt,
                ekf_confidence         = snap_ekf,
                drone_state            = snap_state,
                current_wp             = snap_wp,
                current_pos            = snap_pos,
                signal_loss_count      = snap_loss_count,
                post_reentry           = was_reentry,
                sectors_visited        = snap_visited,
                sectors_remaining      = snap_remaining,
                event_log_snapshot     = snap_log,
                last_loss_duration     = snap_loss_dur,
                last_loss_battery_cost = snap_loss_cost,
                last_loss_drift        = snap_loss_drift,
                pre_loss_intent        = snap_pre_intent,
                pre_loss_wp            = snap_pre_wp,
                intent                 = intent,
                wp_name                = wp_name,
                reasoning              = reasoning,
                confidence             = confidence,
                checkpoint             = checkpoint,
                response_time_seconds  = response_time,
                fallback_used          = fallback_used,
                was_reentry            = was_reentry,
            )
            self.current_record_id = rid
            self.get_logger().info(
                f"[DATASET] Decision #{self.dataset.decision_count} recorded "
                f"(conf={confidence}% fallback={fallback_used})"
            )

            self._print_pilot_decision_banner(
                intent, wp_name, new_target,
                confidence, reasoning, checkpoint,
                fallback_used, was_reentry
            )

        except Exception as e:
            response_time = time.time() - self.llm_query_start_time
            self.get_logger().warn(f"[LLM] Failed: {e} → HOLD_SAFE")
            self.log_event(f"LLM failed: {e} — HOLD_SAFE")
            self.post_signal_loss_reentry = False
            # Overrides NOT cleared on failure — operator must press R
            intent = Intent.HOLD_SAFE
            new_target, wp_name  = self._generate_waypoint(intent)
            self.current_intent  = intent
            self.target_pos      = new_target
            self.current_wp_name = wp_name
            self.drone_state     = DroneState.AUTO_FLYING
            rid = self.dataset.record_decision(
                flight_time_str=self._flight_time_str(),
                battery_pct=snap_batt, ekf_confidence=snap_ekf,
                drone_state=snap_state, current_wp=snap_wp, current_pos=snap_pos,
                signal_loss_count=snap_loss_count, post_reentry=was_reentry,
                sectors_visited=snap_visited, sectors_remaining=snap_remaining,
                event_log_snapshot=snap_log, last_loss_duration=snap_loss_dur,
                last_loss_battery_cost=snap_loss_cost, last_loss_drift=snap_loss_drift,
                pre_loss_intent=snap_pre_intent, pre_loss_wp=snap_pre_wp,
                intent=intent, wp_name=wp_name,
                reasoning=f"LLM failed: {e}. HOLD_SAFE fallback.",
                confidence=99, checkpoint="Until LLM recovers",
                response_time_seconds=response_time, fallback_used=True, was_reentry=was_reentry,
            )
            self.current_record_id = rid

        finally:
            self.llm_running = False

    def _query_llm_intent(self):
        elapsed   = time.time() - self.last_key_time
        visited   = [s['name'] for s in MAP['sectors'] if s['id'] in self.visited_sectors]
        remaining = [s['name'] for s in MAP['sectors'] if s['id'] not in self.visited_sectors]
        unconf    = [t for t in MAP['targets'] if not t['confirmed']]

        sector_str = "\n".join([
            f"  - {s['name']} (priority {s['priority']}, pos={s['pos']}, "
            f"{'✅ VISITED' if s['id'] in self.visited_sectors else '⬜ NOT YET VISITED'})"
            for s in MAP['sectors']
        ])
        target_str = ("\n".join([f"  - Target {t['id']} at {t['pos']} — UNCONFIRMED" for t in unconf])
                      if unconf else "  - All targets confirmed")

        ekf_status = ("GOOD"     if self.ekf_confidence >= 0.75 else
                      "MODERATE" if self.ekf_confidence >= 0.5  else
                      "POOR"     if self.ekf_confidence >= EKF_CONF_MIN else "CRITICAL — do NOT fly far")
        batt_status = ("GOOD"             if self.battery_pct >= BATTERY_LOW_THRESH * 2 else
                       "MODERATE"         if self.battery_pct >= BATTERY_LOW_THRESH     else
                       "LOW — RTB soon"   if self.battery_pct >= BATTERY_CRIT_THRESH    else
                       "CRITICAL — land immediately")

        all_done_note = ""
        if not remaining:
            all_done_note = "\n⚠️  ALL SECTORS VISITED THIS SWEEP. A new sweep will begin — pick the highest priority unvisited sector or RTB if battery low.\n"

        signal_block = self._build_signal_loss_context() if self.post_signal_loss_reentry else ""
        prev_reason  = (f"\nYour previous reasoning: \"{self.last_llm_reasoning}\""
                        if self.last_llm_reasoning else "")

        prompt = f"""You are the autonomous pilot brain of a search-and-rescue UAV.
Human operator is unavailable. Think step by step then decide.

=== CURRENT SITUATION ===
Position  : {[f'{v:.1f}' for v in self.current_pos]}
Battery   : {self.battery_pct:.1f}% — {batt_status}
EKF       : {self.ekf_confidence:.2f} — {ekf_status}
Last human input: {elapsed:.0f}s ago
Currently heading to: {self.current_wp_name}
{signal_block}
=== FLIGHT LOG ===
{self.get_event_log_string()}
{prev_reason}

=== MISSION STATUS ===
Sectors visited   : {visited if visited else 'None yet'}
Sectors remaining : {remaining if remaining else 'ALL DONE — new sweep starting'}
{all_done_note}
Unconfirmed targets:
{target_str}

=== SECTOR MAP ===
{sector_str}
Base      : {MAP['base']}
Comms pt  : {MAP['comms_point']}

=== AVAILABLE ACTIONS ===
1 — HOLD_SAFE              : Loiter in 15m circle at current position
2 — CONTINUE_ASSIGNED_TASK : Resume heading to current waypoint
3 — SEARCH_NEXT_PRIORITY   : Fly to next UNVISITED sector (highest priority first)
4 — INSPECT_TARGET_REPORT  : Fly to nearest unconfirmed victim target
5 — COMMS_REACQUIRE        : Fly to comms recovery point
6 — RETURN_TO_BASE         : Fly home
7 — ABORT_AND_SAFE         : Emergency land now

IMPORTANT RULES:
- For action 3 (SEARCH_NEXT_PRIORITY), always pick from sectors NOT yet visited.
- If battery < {BATTERY_LOW_THRESH}%, seriously consider returning to base.
- If battery < {BATTERY_CRIT_THRESH}%, you MUST land or RTB.
- If EKF < {EKF_CONF_MIN}, do NOT fly far — hold or RTB.
- If signal loss just occurred, address it explicitly before deciding.
- Safety first. Mission second.

Think like an experienced pilot. Respond ONLY with valid JSON:
{{"intent":<1-7>,"reasoning":"<one clear paragraph>","confidence":<0-100>,"next_checkpoint":"<when to reconsider>"}}"""

        result = subprocess.run(
            ["ollama", "run", "llama3"],
            input=prompt, text=True, capture_output=True, timeout=60
        )
        response   = result.stdout.strip()
        json_match = re.search(r'\{[^{}]*"intent"[^{}]*\}', response, re.DOTALL)
        if not json_match:
            raise ValueError(f"No JSON in response: '{response[:150]}'")
        parsed     = json.loads(json_match.group())
        intent_num = int(parsed.get("intent", 1))
        reasoning  = str(parsed.get("reasoning", "No reasoning provided."))
        confidence = max(0, min(100, int(parsed.get("confidence", 50))))
        checkpoint = str(parsed.get("next_checkpoint", "Next LLM cycle"))
        if intent_num not in range(1, 8):
            raise ValueError(f"Intent {intent_num} out of range")
        self.get_logger().info(f"[LLM] intent={intent_num} conf={confidence}%")
        return Intent(intent_num), reasoning, confidence, checkpoint

    # =========================================================
    #  WAYPOINT GENERATOR
    # =========================================================

    def _generate_waypoint(self, intent: Intent):
        def dist3d(pos):
            return math.sqrt(sum((a-b)**2 for a, b in zip(pos, self.current_pos)))
        def safe_alt(z):
            return max(z, -5.0) if self.battery_pct < BATTERY_CRIT_THRESH else z

        if intent == Intent.HOLD_SAFE:
            # FIX 3: HOLD_SAFE always uses loiter circle, never just hover
            self.loitering     = True
            self.circle_center = self.current_pos.copy()
            self.circle_angle  = 0.0
            return self.current_pos.copy(), "LOITER CIRCLE (HOLD_SAFE)"

        elif intent == Intent.CONTINUE_ASSIGNED_TASK:
            self.loitering = False
            s  = next((x for x in MAP['sectors'] if x['id'] == self.assigned_sector), MAP['sectors'][0])
            wp = s['pos'].copy(); wp[2] = safe_alt(wp[2])
            return wp, s['name']

        elif intent == Intent.SEARCH_NEXT_PRIORITY:
            self.loitering = False

            # =====================================================
            #  FIX 1: Use visited_sectors SET to find next sector
            #  Sort all sectors by priority, pick first NOT in visited set
            #  If ALL visited, reset the set and start fresh sweep
            # =====================================================
            all_ids = {s['id'] for s in MAP['sectors']}
            if self.visited_sectors >= all_ids:
                # All sectors visited — reset for a new sweep
                self.visited_sectors = set()
                self.log_event("All sectors visited — resetting for new sweep")
                self.get_logger().info("[MISSION] ♻️  All sectors complete — starting new sweep")

            # Sort by priority (ascending = highest priority first)
            # Break ties by distance, weighted by battery level
            batt_w   = 1.0 + (1.0 - self.battery_pct / 100.0) * 3.0
            sorted_s = sorted(
                MAP['sectors'],
                key=lambda s: s['priority'] + dist3d(s['pos']) * 0.01 * batt_w
            )

            # Pick the first sector NOT already visited
            next_s = next(
                (s for s in sorted_s if s['id'] not in self.visited_sectors),
                sorted_s[0]   # fallback (shouldn't reach here after reset above)
            )

            # Do NOT add to visited_sectors yet — add when drone ARRIVES
            # (so if drone is interrupted before arriving, it retries this sector)
            self.assigned_sector = next_s['id']
            wp = next_s['pos'].copy(); wp[2] = safe_alt(wp[2])
            self.get_logger().info(
                f"[MISSION] Next sector: {next_s['name']} | "
                f"Visited: {len(self.visited_sectors)}/5 | "
                f"Remaining: {5 - len(self.visited_sectors)}"
            )
            return wp, next_s['name']

        elif intent == Intent.INSPECT_TARGET_REPORT:
            self.loitering = False
            unconf = [t for t in MAP['targets'] if not t['confirmed']]
            if not unconf:
                return self._generate_waypoint(Intent.HOLD_SAFE)
            t  = min(unconf, key=lambda t: dist3d(t['pos']))
            wp = t['pos'].copy(); wp[2] = safe_alt(wp[2])
            return wp, f"TARGET_{t['id']}"

        elif intent == Intent.COMMS_REACQUIRE:
            self.loitering = False
            wp = MAP['comms_point'].copy()
            if self.battery_pct < BATTERY_LOW_THRESH: wp[2] = max(wp[2], -15.0)
            return wp, "COMMS_POINT"

        elif intent == Intent.RETURN_TO_BASE:
            self.loitering = False
            wp = MAP['base'].copy()
            if self.battery_pct < BATTERY_CRIT_THRESH: wp[2] = -3.0
            return wp, "BASE"

        elif intent == Intent.ABORT_AND_SAFE:
            self.loitering = False
            self.get_logger().error("[WPT] 🚨 ABORT → EMERGENCY LAND NOW")
            self.send_cmd(21, 0.0)
            wp = self.current_pos.copy(); wp[2] = 0.0
            return wp, "GROUND (EMERGENCY LAND)"

        return self.current_pos.copy(), "UNKNOWN"

    # =========================================================
    #  LOITER UPDATE
    # =========================================================

    def update_loiter(self):
        if not self.loitering or self.circle_center is None: return
        self.circle_angle += LOITER_OMEGA * self.timer_period
        cx, cy, cz = self.circle_center
        self.target_pos[0] = cx + LOITER_RADIUS * math.cos(self.circle_angle)
        self.target_pos[1] = cy + LOITER_RADIUS * math.sin(self.circle_angle)
        self.target_pos[2] = cz

    # =========================================================
    #  MAIN LOOP (20 Hz)
    # =========================================================

    def loop(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        now       = time.time()

        # Offboard heartbeat — must send every loop or PX4 drops offboard mode
        offboard           = OffboardControlMode()
        offboard.timestamp = timestamp
        offboard.position  = True
        self.offboard_pub.publish(offboard)

        comm_loss = self.check_comm_loss()

        # ---- Signal loss / regain detection ----
        if comm_loss and not self.signal_was_lost:
            self.signal_was_lost      = True
            self.signal_lost_time     = now
            self.signal_lost_battery  = self.battery_pct
            self.signal_lost_position = self.current_pos.copy()
            self.pre_loss_intent      = self.current_intent
            self.pre_loss_wp_name     = self.current_wp_name
            self.pre_loss_battery     = self.battery_pct
            self.pre_loss_position    = self.current_pos.copy()
            self.signal_loss_count   += 1
            self.signal_loss_since_depart = True
            self.log_event(
                f"Signal/comm loss #{self.signal_loss_count}. "
                f"Battery: {self.battery_pct:.1f}%. Was heading: {self.pre_loss_wp_name}"
            )
            self._print_signal_banner(
                "LOST",
                f"Loss #{self.signal_loss_count} | Batt: {self.battery_pct:.1f}% | Was: {self.pre_loss_wp_name}"
            )

        elif not comm_loss and self.signal_was_lost:
            duration = now - self.signal_lost_time
            self.last_loss_duration     = duration
            self.last_loss_battery_cost = self.signal_lost_battery - self.battery_pct
            if self.signal_lost_position:
                dx = self.current_pos[0] - self.signal_lost_position[0]
                dy = self.current_pos[1] - self.signal_lost_position[1]
                self.last_loss_drift = math.sqrt(dx*dx + dy*dy)
            else:
                self.last_loss_drift = 0.0
            self.signal_was_lost         = False
            self.post_signal_loss_reentry = True
            self.log_event(
                f"Signal regained. Duration: {duration:.1f}s. "
                f"Cost: {self.last_loss_battery_cost:.1f}%. Drift: {self.last_loss_drift:.1f}m"
            )
            self._print_signal_banner(
                "REGAINED",
                f"Duration: {duration:.1f}s | Cost: {self.last_loss_battery_cost:.1f}% | Drift: {self.last_loss_drift:.1f}m"
            )
            self.last_llm_time = 0.0
            self.drone_state   = DroneState.AUTO_INIT

        # ---- State machine ----
        if not comm_loss:
            if self.drone_state != DroneState.MANUAL and not self.post_signal_loss_reentry:
                self.drone_state    = DroneState.MANUAL
                self.loitering      = False
                self.current_intent = None
                self.dwell_active   = False
        else:
            if self.drone_state == DroneState.MANUAL:
                self.drone_state   = DroneState.AUTO_INIT
                self.last_llm_time = 0.0
                elapsed_no_input   = now - self.last_key_time
                self.log_event(
                    f"Switched to AUTONOMOUS. No input: {elapsed_no_input:.0f}s. "
                    f"Battery: {self.battery_pct:.1f}%. EKF: {self.ekf_confidence:.2f}"
                )
                self._print_mode_change("AUTONOMOUS", f"No input for {elapsed_no_input:.0f}s")

            if self.drone_state == DroneState.AUTO_INIT:
                if not self.llm_running:
                    self.trigger_llm()

            elif self.drone_state == DroneState.AUTO_FLYING:
                if not self.loitering and self.check_waypoint_reached():
                    # =====================================================
                    #  FIX 1: Mark sector as VISITED when drone ARRIVES
                    #  This is the correct place — not when decision is made.
                    #  If drone is interrupted on the way, sector stays unvisited.
                    # =====================================================
                    arrived_sector = next(
                        (s for s in MAP['sectors'] if s['name'] == self.current_wp_name), None
                    )
                    if arrived_sector and arrived_sector['id'] not in self.visited_sectors:
                        self.visited_sectors.add(arrived_sector['id'])
                        visited_count = len(self.visited_sectors)
                        self.log_event(
                            f"Arrived and marked {self.current_wp_name} as visited. "
                            f"({visited_count}/5 sectors done)"
                        )
                        self.get_logger().info(
                            f"[MISSION] ✅ {self.current_wp_name} visited "
                            f"({visited_count}/5) | "
                            f"Remaining: {[s['name'] for s in MAP['sectors'] if s['id'] not in self.visited_sectors]}"
                        )
                    # =====================================================

                    self.drone_state  = DroneState.AUTO_DWELLING
                    self.dwell_start  = now
                    self.dwell_active = True
                    self.dwell_logged = False
                    time_to_arrive    = now - self.waypoint_depart_time
                    self.log_event(
                        f"Arrived at {self.current_wp_name}. "
                        f"Battery: {self.battery_pct:.1f}%. Time: {time_to_arrive:.0f}s."
                    )
                    self._print_arrived_banner(self.current_wp_name)

                    if self.current_record_id:
                        self.dataset.update_outcome(
                            rid                 = self.current_record_id,
                            waypoint_reached    = True,
                            battery_at_arrival  = self.battery_pct,
                            ekf_at_arrival      = self.ekf_confidence,
                            time_to_arrive      = time_to_arrive,
                            another_signal_loss = self.signal_loss_since_depart,
                            dwell_completed     = False,
                        )

                elif not self.llm_running and (now - self.last_llm_time) > INTENT_REQUERY_INTERVAL:
                    self.log_event(f"LLM re-query (30s). Still heading to {self.current_wp_name}.")
                    if self.current_record_id:
                        self.dataset.update_outcome(
                            rid                 = self.current_record_id,
                            waypoint_reached    = False,
                            battery_at_arrival  = self.battery_pct,
                            ekf_at_arrival      = self.ekf_confidence,
                            time_to_arrive      = now - self.waypoint_depart_time,
                            another_signal_loss = self.signal_loss_since_depart,
                            dwell_completed     = False,
                        )
                    self.trigger_llm()

            elif self.drone_state == DroneState.AUTO_DWELLING:
                elapsed_dwell = now - self.dwell_start
                remaining     = DWELL_TIME - elapsed_dwell
                self.dwell_remaining = max(0.0, remaining)

                if int(elapsed_dwell) % 5 == 0 and not self.dwell_logged:
                    self.dwell_logged = True
                    self.get_logger().info(f"[DWELL] ⏳ {self.current_wp_name} | {remaining:.0f}s remaining")
                elif int(elapsed_dwell) % 5 != 0:
                    self.dwell_logged = False

                if remaining <= 0:
                    self.log_event(f"Dwell complete at {self.current_wp_name}. Battery: {self.battery_pct:.1f}%")
                    if self.current_record_id:
                        self.dataset.update_outcome(
                            rid                 = self.current_record_id,
                            waypoint_reached    = True,
                            battery_at_arrival  = self.battery_pct,
                            ekf_at_arrival      = self.ekf_confidence,
                            time_to_arrive      = now - self.waypoint_depart_time,
                            another_signal_loss = self.signal_loss_since_depart,
                            dwell_completed     = True,
                        )
                    self.drone_state   = DroneState.AUTO_INIT
                    self.dwell_active  = False
                    self.last_llm_time = 0.0
                    self.trigger_llm()

            self.update_loiter()

        # ---- Send trajectory setpoint ----
        sp           = TrajectorySetpoint()
        sp.timestamp = timestamp
        sp.position  = [float(v) for v in self.target_pos]
        sp.yaw       = (self.circle_angle + math.pi / 2.0 if self.loitering else 0.0)
        self.traj_pub.publish(sp)

        # ---- Arm sequence ----
        if self.counter == 100:
            self.send_cmd(176, 1.0, 6.0)   # enable offboard
            self.log_event("Offboard mode command sent.")
        if self.counter == 120:
            self.send_cmd(400, 1.0)         # ARM
            self.log_event("ARM command sent.")

        # ---- Dashboard ----
        if self.counter % 10 == 0:
            self._write_dashboard_state()

        # ---- Status print ----
        if self.counter % 50 == 0:
            ds        = self.dataset.get_stats()
            intent_s  = self.current_intent.name if self.current_intent else "NONE"
            visited_n = [s['name'] for s in MAP['sectors'] if s['id'] in self.visited_sectors]
            remain_n  = [s['name'] for s in MAP['sectors'] if s['id'] not in self.visited_sectors]
            ovr_tag   = ""
            if self.ekf_override:     ovr_tag += " [EKF_OVR]"
            if self.battery_override: ovr_tag += " [BATT_OVR]"
            self.get_logger().info(
                f"[STATUS] {self.drone_state.value}{ovr_tag} | "
                f"Intent={intent_s} conf={self.last_llm_confidence}% | "
                f"WP={self.current_wp_name} | "
                f"Batt={self.battery_pct:.1f}% | EKF={self.ekf_confidence:.2f} | "
                f"Visited={visited_n} Remaining={remain_n} | "
                f"Dataset={ds['total_decisions']}"
            )

        self.counter += 1

    # =========================================================
    #  COMMAND SENDER
    # =========================================================

    def send_cmd(self, command, p1=0.0, p2=0.0):
        msg               = VehicleCommand()
        msg.timestamp     = int(self.get_clock().now().nanoseconds / 1000)
        msg.command       = command
        msg.param1        = float(p1)
        msg.param2        = float(p2)
        msg.target_system = 1; msg.target_component = 1
        msg.source_system = 1; msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)


# ============================================================
#  ENTRY POINT
# ============================================================
def main():
    rclpy.init()
    node = UAVAutonomousNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[SYS] Shutting down...")
        ds = node.dataset.get_stats()
        node.get_logger().info(
            f"[DATASET] Session done. Decisions={ds['total_decisions']} "
            f"Fallbacks={ds['fallback_count']} File={ds['filepath']}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

