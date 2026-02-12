#!/usr/bin/env python3
"""
UAV Autonomous Control System v3
==================================
New in v3:
  1. AUTO-START: From launch, if no human input after 15s,
     drone immediately picks highest-priority sector and flies there.
     No need for operator to do anything.
  2. DWELL: After reaching every waypoint, drone waits 45 seconds
     at that location (hovering in place) before querying LLM
     for the next intent.
  3. CLEAR INTENT DISPLAY: Every intent decision is printed
     in a big visible banner in the terminal.
  4. DWELL COUNTDOWN: Terminal shows countdown timer while waiting.

Behavior timeline from launch:
  t=0s   : Script starts, drone arms and hovers
  t=0-15s: Waiting for human input
  t=15s  : No human → AUTO mode → LLM picks SECTOR_A (priority 1)
  t=...  : Drone flies to SECTOR_A
  t=arr  : Drone arrives → "ARRIVED" banner → waits 45s
  t=arr+45s: LLM picks next intent → drone flies there
  ...and so on forever until human takes control or battery dies
"""

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
import math
import threading
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


# Intent descriptions for terminal display
INTENT_DESC = {
    Intent.HOLD_SAFE:              "Loiter safely at current position",
    Intent.CONTINUE_ASSIGNED_TASK: "Continue last acknowledged sector sweep",
    Intent.SEARCH_NEXT_PRIORITY:   "Fly to next highest-priority sector",
    Intent.INSPECT_TARGET_REPORT:  "Fly to confirm nearest target/victim",
    Intent.COMMS_REACQUIRE:        "Fly to comms-friendly location",
    Intent.RETURN_TO_BASE:         "Return to base (battery/time)",
    Intent.ABORT_AND_SAFE:         "EMERGENCY - Land immediately",
}


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
KEYBOARD_TIMEOUT      = 15.0   # seconds no input → auto mode starts
BATTERY_LOW_THRESH    = 25.0   # % → comm loss trigger + route adjustment
BATTERY_CRIT_THRESH   = 15.0   # % → force RTB or ABORT
EKF_CONF_MIN          = 0.4    # below this → comm loss trigger
COMMLOSS_SCORE_THRESH = 1      # 1=keyboard alone triggers, 2=needs 2 triggers
WAYPOINT_REACH_DIST   = 3.0    # meters - how close = "arrived"
DWELL_TIME            = 20.0   # seconds to wait at each waypoint
INTENT_REQUERY_INTERVAL = 30.0 # re-query LLM every 30s if not arrived yet


# ============================================================
#  DRONE STATES
# ============================================================
class DroneState(Enum):
    MANUAL        = "MANUAL"          # Human is in control
    AUTO_FLYING   = "AUTO_FLYING"     # Flying to a waypoint autonomously
    AUTO_DWELLING = "AUTO_DWELLING"   # Waiting at waypoint (45s dwell)
    AUTO_INIT     = "AUTO_INIT"       # Just entered auto, LLM not queried yet


# ============================================================
#  MAIN NODE
# ============================================================
class UAVAutonomousNode(Node):

    def __init__(self):
        super().__init__('uav_autonomous_node')

        # ---- QoS: VOLATILE (position, EKF) ----
        qos_volatile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ---- QoS: TRANSIENT_LOCAL (battery, status) ----
        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ---------------- Publishers ----------------
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # ---------------- Subscribers ----------------
        # Try _v1 topic first - this is what PX4 actually publishes via DDS
        # If _v1 doesn't work either, run:
        #   ros2 topic list | grep vehicle_local
        # and update the topic name below accordingly
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.pos_cb, qos_volatile)

        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status_v1',
            self.battery_cb, qos_transient)

        self.ekf_sub = self.create_subscription(
            EstimatorStatusFlags,
            '/fmu/out/estimator_status_flags',
            self.ekf_cb, qos_volatile)

        # ---------------- Position ----------------
        self.current_pos     = [0.0, 0.0, -5.0]
        self.target_pos      = [0.0, 0.0, -5.0]
        self.pos_initialized = False

        # ---------------- Battery ----------------
        self.battery_pct     = 100.0
        self.battery_volt    = 0.0
        self.battery_amp     = 0.0

        # ---------------- EKF ----------------
        self.ekf_confidence  = 1.0

        # ---------------- Keyboard ----------------
        self.last_key_time   = time.time()

        # ---------------- Drone State Machine ----------------
        self.drone_state     = DroneState.MANUAL
        self.current_intent  = None
        self.current_wp_name = "None"   # Human-readable name of current target

        # ---------------- LLM (background thread) ----------------
        self.llm_running     = False
        self.llm_thread      = None
        self.last_llm_time   = 0.0

        # ---------------- Dwell State ----------------
        self.dwell_start     = 0.0      # When drone arrived at waypoint
        self.dwell_active    = False    # True while in 45s dwell period
        self.dwell_logged    = False    # To avoid repeating arrival log

        # ---------------- Loiter (HOLD_SAFE circle) ----------------
        self.loitering       = False
        self.circle_radius   = 15.0
        self.circle_omega    = 0.3
        self.circle_angle    = 0.0
        self.circle_center   = None

        # ---------------- Mission ----------------
        self.last_sector_id  = -1       # -1 means no sector visited yet
        self.assigned_sector = 0

        # ---------------- Loop ----------------
        self.counter         = 0
        self.timer_period    = 0.05     # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.loop)

        # ---------------- Keyboard Listener ----------------
        kbl = keyboard.Listener(on_press=self.on_key_press)
        kbl.daemon = True
        kbl.start()

        self._print_startup_banner()

    # =========================================================
    #  TERMINAL DISPLAY HELPERS
    # =========================================================

    def _print_startup_banner(self):
        log = self.get_logger().info
        log("╔══════════════════════════════════════════════════════════╗")
        log("║         UAV AUTONOMOUS SYSTEM v3 - STARTED              ║")
        log("║                                                          ║")
        log("║  AUTO-START: No input for 15s → flies to SECTOR_A       ║")
        log("║  DWELL: Waits 45s at each waypoint                       ║")
        log("║                                                          ║")
        log("║  MOVEMENT KEYS:                                          ║")
        log("║  Arrows/W/S = manual move | B = battery | H = help      ║")
        log("║                                                          ║")
        log("║  TEST KEYS:                                              ║")
        log("║  T = bad EKF → HOLD_SAFE    Y = low batt → RTB          ║")
        log("║  R = reset normal           F = force LLM now           ║")
        log("║  1-7 = force specific intent directly                   ║")
        log("╚══════════════════════════════════════════════════════════╝")

    def _print_intent_banner(self, intent: Intent, target_name: str, target_pos: list):
        """Prints a large visible banner when a new intent is chosen"""
        log  = self.get_logger().info
        desc = INTENT_DESC.get(intent, "Unknown")
        pos_str = [f'{v:.1f}' for v in target_pos]

        log("╔══════════════════════════════════════════════════════════╗")
        log(f"║  🤖 LLM DECISION                                        ║")
        log(f"║  Intent  : [{intent.value}] {intent.name:<35}║")
        log(f"║  Action  : {desc:<47}║")
        log(f"║  Target  : {target_name:<47}║")
        log(f"║  Position: {str(pos_str):<47}║")
        log(f"║  Battery : {self.battery_pct:.1f}%  |  EKF: {self.ekf_confidence:.2f}                     ║")
        log("╚══════════════════════════════════════════════════════════╝")

    def _print_arrived_banner(self, wp_name: str):
        """Prints arrival notice with 45s dwell countdown info"""
        log = self.get_logger().info
        log("╔══════════════════════════════════════════════════════════╗")
        log(f"║  ✅ ARRIVED AT: {wp_name:<43}║")
        log(f"║  Hovering for {DWELL_TIME:.0f}s ... then querying LLM for next     ║")
        log("╚══════════════════════════════════════════════════════════╝")

    def _print_dwell_countdown(self, remaining: float):
        """Prints dwell countdown every 5 seconds"""
        self.get_logger().info(
            f"[DWELL] ⏳ Waiting at {self.current_wp_name} | "
            f"{remaining:.0f}s remaining before next intent"
        )

    def _print_mode_change(self, new_mode: str, reason: str):
        log = self.get_logger().warn
        log(f"┌─────────────────────────────────────────────────────────┐")
        log(f"│  MODE CHANGE → {new_mode:<42}│")
        log(f"│  Reason: {reason:<48}│")
        log(f"└─────────────────────────────────────────────────────────┘")

    # =========================================================
    #  SUBSCRIBER CALLBACKS
    # =========================================================

    def pos_cb(self, msg):
        self.current_pos = [msg.x, msg.y, msg.z]
        if not self.pos_initialized:
            self.target_pos      = self.current_pos.copy()
            self.pos_initialized = True
            self.get_logger().info(
                f"[POS] ✓ Position initialized: {[f'{v:.1f}' for v in self.current_pos]}"
            )

    def battery_cb(self, msg):
        self.battery_pct  = msg.remaining * 100.0
        self.battery_volt = msg.voltage_v
        self.battery_amp  = msg.current_a

        if self.battery_pct < BATTERY_CRIT_THRESH and self.counter % 40 == 0:
            self.get_logger().error(
                f"[BATT] 🚨 CRITICAL {self.battery_pct:.1f}% - RTB RECOMMENDED"
            )
        elif self.battery_pct < BATTERY_LOW_THRESH and self.counter % 200 == 0:
            self.get_logger().warn(f"[BATT] ⚠ LOW {self.battery_pct:.1f}%")

    def ekf_cb(self, msg):
        penalty = 0.0
        if hasattr(msg, 'cs_tilt_align') and not msg.cs_tilt_align: penalty += 0.4
        if hasattr(msg, 'cs_yaw_align')  and not msg.cs_yaw_align:  penalty += 0.4
        if hasattr(msg, 'cs_gps')        and not msg.cs_gps:        penalty += 0.2
        if hasattr(msg, 'cs_baro_hgt')   and not msg.cs_baro_hgt:   penalty += 0.2
        if hasattr(msg, 'cs_opt_flow')   and not msg.cs_opt_flow:   penalty += 0.1
        if hasattr(msg, 'cs_mag_hdg')    and not msg.cs_mag_hdg:    penalty += 0.1
        self.ekf_confidence = max(0.0, min(1.0, 1.0 - penalty))

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
            elif key.char == 'w':           self.target_pos[2] -= step; moved = True
            elif key.char == 's':           self.target_pos[2] += step; moved = True
            elif key.char == 'b':
                self.get_logger().info(
                    f"[BATT] {self.battery_pct:.1f}% | "
                    f"{self.battery_volt:.2f}V | {self.battery_amp:.2f}A"
                )
                return
            elif key.char == 'h':
                self.get_logger().info("=" * 55)
                self.get_logger().info("  KEYS:")
                self.get_logger().info("  Arrows / W / S = manual movement")
                self.get_logger().info("  B = battery status")
                self.get_logger().info("  H = this help menu")
                self.get_logger().info("  --- TEST KEYS ---")
                self.get_logger().info("  T = force bad EKF (0.2) → triggers HOLD_SAFE")
                self.get_logger().info("  Y = force low battery (10%) → triggers RTB")
                self.get_logger().info("  R = reset EKF + battery to normal")
                self.get_logger().info("  F = force immediate LLM re-query")
                self.get_logger().info("  1-7 = force specific intent directly")
                self.get_logger().info("=" * 55)
                return

            # ---- TEST KEY: T = force bad EKF → HOLD_SAFE ----
            elif key.char == 't':
                self.ekf_confidence = 0.2
                self.get_logger().warn(
                    "[TEST] ⚠ EKF forced to 0.2 (CRITICAL) "
                    "→ LLM should pick HOLD_SAFE (intent 1)"
                )
                # Force immediate re-query
                self.last_llm_time = 0.0
                self.drone_state   = DroneState.AUTO_INIT
                return

            # ---- TEST KEY: Y = force low battery → RTB ----
            elif key.char == 'y':
                self.battery_pct = 10.0
                self.get_logger().warn(
                    "[TEST] ⚠ Battery forced to 10% (CRITICAL) "
                    "→ LLM should pick RETURN_TO_BASE (intent 6)"
                )
                # Force immediate re-query
                self.last_llm_time = 0.0
                self.drone_state   = DroneState.AUTO_INIT
                return

            # ---- TEST KEY: R = reset everything to normal ----
            elif key.char == 'r':
                self.ekf_confidence = 1.0
                self.battery_pct    = 100.0
                self.get_logger().info(
                    "[TEST] ✓ Reset: EKF=1.0, Battery=100% "
                    "→ Normal operation restored"
                )
                # Force re-query with fresh state
                self.last_llm_time = 0.0
                self.drone_state   = DroneState.AUTO_INIT
                return

            # ---- TEST KEY: F = force immediate LLM re-query ----
            elif key.char == 'f':
                self.last_llm_time = 0.0
                self.drone_state   = DroneState.AUTO_INIT
                self.dwell_active  = False
                self.get_logger().warn(
                    "[TEST] 🔄 Forcing immediate LLM re-query..."
                )
                return

            # ---- TEST KEYS: 1-7 = force specific intent directly ----
            elif key.char in '1234567':
                intent_num = int(key.char)
                intent     = Intent(intent_num)
                self.get_logger().warn(
                    f"[TEST] 🎯 Forcing intent {intent_num}: {intent.name}"
                )
                # Generate waypoint for forced intent
                new_target, wp_name = self._generate_waypoint(intent)
                self.current_intent  = intent
                self.target_pos      = new_target
                self.current_wp_name = wp_name
                self.drone_state     = DroneState.AUTO_FLYING
                self.dwell_active    = False
                self._print_intent_banner(intent, wp_name, new_target)
                return

            if moved:
                self.last_key_time  = time.time()
                self.loitering      = False
                self.circle_center  = None
                self.dwell_active   = False

                # Switch back to MANUAL if we were in AUTO
                if self.drone_state != DroneState.MANUAL:
                    self.drone_state    = DroneState.MANUAL
                    self.current_intent = None
                    self._print_mode_change(
                        "MANUAL", "Human operator took control"
                    )

                self.get_logger().info(
                    f"[KEY] Target → {[f'{v:.1f}' for v in self.target_pos]}"
                )

        except AttributeError:
            pass

    # =========================================================
    #  COMM LOSS DETECTOR
    # =========================================================

    def check_comm_loss(self) -> bool:
        score   = 0
        reasons = []

        elapsed = time.time() - self.last_key_time
        if elapsed > KEYBOARD_TIMEOUT:
            score += 1
            reasons.append(f"no_key={elapsed:.0f}s")

        if self.battery_pct < BATTERY_LOW_THRESH:
            score += 1
            reasons.append(f"batt={self.battery_pct:.0f}%")

        if self.ekf_confidence < EKF_CONF_MIN:
            score += 1
            reasons.append(f"ekf={self.ekf_confidence:.2f}")

        if score >= COMMLOSS_SCORE_THRESH:
            return True

        return False

    # =========================================================
    #  WAYPOINT REACHED
    # =========================================================

    def check_waypoint_reached(self) -> bool:
        if not self.pos_initialized:
            return False
        dx = self.current_pos[0] - self.target_pos[0]
        dy = self.current_pos[1] - self.target_pos[1]
        return math.sqrt(dx*dx + dy*dy) < WAYPOINT_REACH_DIST

    # =========================================================
    #  LLM BLOCK  (background thread)
    # =========================================================

    def trigger_llm(self):
        """Launch LLM query in background thread - doesn't block loop"""
        if self.llm_running:
            return
        self.llm_running = True
        self.get_logger().info("[LLM] 🔄 Querying Ollama for next intent...")
        t = threading.Thread(target=self._llm_worker, daemon=True)
        t.start()

    def _llm_worker(self):
        """Runs in background. Picks intent + waypoint, updates state."""
        try:
            intent     = self._query_llm_intent()
            new_target, wp_name = self._generate_waypoint(intent)

            # Update state
            self.current_intent  = intent
            self.target_pos      = new_target
            self.current_wp_name = wp_name
            self.last_llm_time   = time.time()
            self.dwell_active    = False
            self.dwell_logged    = False
            self.drone_state     = DroneState.AUTO_FLYING

            # Big terminal banner
            self._print_intent_banner(intent, wp_name, new_target)

        except Exception as e:
            self.get_logger().warn(f"[LLM] Failed: {e} → defaulting HOLD_SAFE")
            self.current_intent = Intent.HOLD_SAFE
            self.drone_state    = DroneState.AUTO_FLYING

        finally:
            self.llm_running = False

    def _query_llm_intent(self) -> Intent:
        elapsed = time.time() - self.last_key_time

        sector_str = "\n".join([
            f"  {s['id']}: {s['name']} priority={s['priority']} pos={s['pos']}"
            for s in MAP['sectors']
        ])
        target_str = "\n".join([
            f"  {t['id']}: pos={t['pos']} confirmed={t['confirmed']}"
            for t in MAP['targets']
        ])

        # Build history string so LLM knows what was already tried
        history_str = (
            f"Last intent: {self.current_intent.name if self.current_intent else 'NONE'}\n"
            f"Last waypoint visited: {self.current_wp_name}\n"
            f"Last sector ID visited: {self.last_sector_id}\n"
            f"Times visited COMMS_POINT already: "
            f"{'YES - already tried, move on!' if self.current_wp_name == 'COMMS_POINT' else 'NO'}"
        )

        # EKF plain english explanation
        if self.ekf_confidence >= 0.75:
            ekf_status = "GOOD - position estimate is reliable"
        elif self.ekf_confidence >= 0.5:
            ekf_status = "MODERATE - some sensors degraded"
        elif self.ekf_confidence >= 0.3:
            ekf_status = "POOR - position drift possible"
        else:
            ekf_status = "CRITICAL - position unreliable, do not fly far"

        # Battery plain english - uses actual threshold variables
        if self.battery_pct >= BATTERY_LOW_THRESH * 2:
            batt_status = "GOOD - plenty of flight time remaining"
        elif self.battery_pct >= BATTERY_LOW_THRESH:
            batt_status = "MODERATE - limit long-distance flights"
        elif self.battery_pct >= BATTERY_CRIT_THRESH:
            batt_status = f"LOW - return to base soon (below {BATTERY_LOW_THRESH}%)"
        else:
            batt_status = f"CRITICAL - land immediately (below {BATTERY_CRIT_THRESH}%)"

        prompt = f"""You are an autonomous UAV search-and-rescue decision system.
Human operator has lost communication. You must choose the next action.

=== CURRENT STATE ===
Position:         {[f'{v:.1f}' for v in self.current_pos]}
Battery:          {self.battery_pct:.1f}% → {batt_status}
EKF Confidence:   {self.ekf_confidence:.2f} → {ekf_status}
No human input:   {elapsed:.0f} seconds ago

=== MISSION HISTORY (do NOT repeat same action) ===
{history_str}

=== MAP ===
Sectors to search (lower priority number = search first):
{sector_str}

Unconfirmed victim targets:
{target_str}

Base (home):    {MAP['base']}
Comms point:    {MAP['comms_point']}

=== AVAILABLE INTENTS ===
1: HOLD_SAFE              - hover in place, do nothing
2: CONTINUE_ASSIGNED_TASK - go to assigned sector {self.assigned_sector}
3: SEARCH_NEXT_PRIORITY   - fly to next unvisited priority sector (PREFERRED during search)
4: INSPECT_TARGET_REPORT  - fly to nearest unconfirmed victim target
5: COMMS_REACQUIRE        - fly to comms recovery point (only if not tried yet)
6: RETURN_TO_BASE         - fly home
7: ABORT_AND_SAFE         - emergency land NOW

=== STRICT DECISION RULES (follow in order) ===
RULE 1: Battery CRITICAL (<{BATTERY_CRIT_THRESH}%)  → MUST pick 6 or 7
RULE 2: EKF CRITICAL (<{EKF_CONF_MIN})              → MUST pick 1 (hover, do not move)
RULE 3: Already visited COMMS_POINT this session    → do NOT pick 5 again, pick 3
RULE 4: Battery GOOD and EKF GOOD and no input > {KEYBOARD_TIMEOUT}s → pick 3
RULE 5: Unconfirmed targets exist and battery > 40% → consider picking 4
RULE 6: Battery LOW (<{BATTERY_LOW_THRESH}%)        → pick 6 (return to base)
RULE 7: If none of the above                        → pick 3 (keep searching)

Current battery {self.battery_pct:.1f}% vs CRITICAL threshold {BATTERY_CRIT_THRESH}%:
{"→ BATTERY IS CRITICAL! MUST pick 6 or 7!" if self.battery_pct < BATTERY_CRIT_THRESH else "→ Battery ok, continue mission"}

Current EKF {self.ekf_confidence:.2f} vs minimum {EKF_CONF_MIN}:
{"→ EKF IS CRITICAL! MUST pick 1!" if self.ekf_confidence < EKF_CONF_MIN else "→ EKF ok, safe to fly"}

IMPORTANT: The mission is SEARCH AND RESCUE. Default behavior is to SEARCH (intent 3).
Only deviate from searching if safety rules above require it.

Reply with ONLY a single digit 1-7. Nothing else."""

        result = subprocess.run(
            ["ollama", "run", "llama3"],
            input=prompt, text=True,
            capture_output=True, timeout=45
        )

        response = result.stdout.strip()
        self.get_logger().info(f"[LLM] Raw response: '{response}'")

        match = re.search(r'\b([1-7])\b', response)
        if not match:
            raise ValueError(f"No valid digit in: '{response}'")

        return Intent(int(match.group()))

    # =========================================================
    #  WAYPOINT GENERATOR
    # =========================================================

    def _generate_waypoint(self, intent: Intent):
        """
        Returns (waypoint [x,y,z], name string)
        Battery-aware optimization for each intent.
        """

        def dist3d(pos):
            return math.sqrt(
                (pos[0]-self.current_pos[0])**2 +
                (pos[1]-self.current_pos[1])**2 +
                (pos[2]-self.current_pos[2])**2
            )

        def safe_alt(z):
            if self.battery_pct < BATTERY_CRIT_THRESH:
                return max(z, -5.0)
            return z

        # ---- 1. HOLD_SAFE ----
        if intent == Intent.HOLD_SAFE:
            self.loitering     = True
            self.circle_center = self.current_pos.copy()
            self.circle_angle  = 0.0
            return self.current_pos.copy(), "CURRENT POSITION (loiter)"

        # ---- 2. CONTINUE_ASSIGNED_TASK ----
        elif intent == Intent.CONTINUE_ASSIGNED_TASK:
            self.loitering = False
            s  = next((x for x in MAP['sectors'] if x['id'] == self.assigned_sector),
                      MAP['sectors'][0])
            wp = s['pos'].copy()
            wp[2] = safe_alt(wp[2])
            return wp, s['name']

        # ---- 3. SEARCH_NEXT_PRIORITY ----
        elif intent == Intent.SEARCH_NEXT_PRIORITY:
            self.loitering = False
            batt_w = 1.0 + (1.0 - self.battery_pct / 100.0) * 3.0
            sorted_s = sorted(
                MAP['sectors'],
                key=lambda s: s['priority'] + dist3d(s['pos']) * 0.01 * batt_w
            )
            next_s = next(
                (s for s in sorted_s if s['id'] != self.last_sector_id),
                sorted_s[0]
            )
            self.last_sector_id = next_s['id']
            wp = next_s['pos'].copy()
            wp[2] = safe_alt(wp[2])
            return wp, next_s['name']

        # ---- 4. INSPECT_TARGET_REPORT ----
        elif intent == Intent.INSPECT_TARGET_REPORT:
            self.loitering = False
            unconf = [t for t in MAP['targets'] if not t['confirmed']]
            if not unconf:
                self.get_logger().warn("[WPT] No unconfirmed targets → HOLD_SAFE")
                return self._generate_waypoint(Intent.HOLD_SAFE)
            t  = min(unconf, key=lambda t: dist3d(t['pos']))
            wp = t['pos'].copy()
            wp[2] = safe_alt(wp[2])
            return wp, f"TARGET_{t['id']}"

        # ---- 5. COMMS_REACQUIRE ----
        elif intent == Intent.COMMS_REACQUIRE:
            self.loitering = False
            wp = MAP['comms_point'].copy()
            if self.battery_pct < BATTERY_LOW_THRESH:
                wp[2] = max(wp[2], -15.0)
            return wp, "COMMS_POINT"

        # ---- 6. RETURN_TO_BASE ----
        elif intent == Intent.RETURN_TO_BASE:
            self.loitering = False
            wp = MAP['base'].copy()
            if self.battery_pct < BATTERY_CRIT_THRESH:
                wp[2] = -3.0
            return wp, "BASE"

        # ---- 7. ABORT_AND_SAFE ----
        elif intent == Intent.ABORT_AND_SAFE:
            self.loitering = False
            self.get_logger().error("[WPT] 🚨 ABORT → LANDING NOW")
            self.send_cmd(21, 0.0)   # MAV_CMD_NAV_LAND
            wp = self.current_pos.copy()
            wp[2] = 0.0
            return wp, "GROUND (EMERGENCY LAND)"

        return self.current_pos.copy(), "UNKNOWN"

    # =========================================================
    #  LOITER UPDATE
    # =========================================================

    def update_loiter(self):
        if not self.loitering or self.circle_center is None:
            return
        self.circle_angle  += self.circle_omega * self.timer_period
        cx, cy, cz          = self.circle_center
        self.target_pos[0]  = cx + self.circle_radius * math.cos(self.circle_angle)
        self.target_pos[1]  = cy + self.circle_radius * math.sin(self.circle_angle)
        self.target_pos[2]  = cz

    # =========================================================
    #  MAIN CONTROL LOOP  (20 Hz)
    # =========================================================

    def loop(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        now       = time.time()

        # ---- Offboard heartbeat (must be >2Hz) ----
        offboard           = OffboardControlMode()
        offboard.timestamp = timestamp
        offboard.position  = True
        self.offboard_pub.publish(offboard)

        # ---- Comm loss check ----
        comm_loss = self.check_comm_loss()

        # ======================================================
        #  STATE MACHINE
        # ======================================================

        if not comm_loss:
            # ------------------------------------------------
            #  MANUAL MODE
            # ------------------------------------------------
            if self.drone_state != DroneState.MANUAL:
                self.drone_state    = DroneState.MANUAL
                self.loitering      = False
                self.current_intent = None
                self.dwell_active   = False

        else:
            # ------------------------------------------------
            #  AUTONOMOUS MODE
            # ------------------------------------------------

            # First frame entering AUTO from MANUAL
            if self.drone_state == DroneState.MANUAL:
                self.drone_state  = DroneState.AUTO_INIT
                self.last_llm_time = 0.0   # Force immediate LLM query
                self._print_mode_change(
                    "AUTONOMOUS",
                    f"No human input for {time.time()-self.last_key_time:.0f}s"
                )

            # AUTO_INIT or time to re-query LLM
            if self.drone_state == DroneState.AUTO_INIT:
                if not self.llm_running:
                    self.trigger_llm()

            # AUTO_FLYING: check if arrived
            elif self.drone_state == DroneState.AUTO_FLYING:
                if not self.loitering and self.check_waypoint_reached():
                    # Just arrived!
                    self.drone_state  = DroneState.AUTO_DWELLING
                    self.dwell_start  = now
                    self.dwell_active = True
                    self.dwell_logged = False
                    self._print_arrived_banner(self.current_wp_name)

                # Safety re-query if flying for too long without arriving
                elif not self.llm_running and (now - self.last_llm_time) > INTENT_REQUERY_INTERVAL:
                    self.get_logger().info(
                        "[LLM] Re-querying (still flying, 30s passed)..."
                    )
                    self.trigger_llm()

            # AUTO_DWELLING: 45s wait at waypoint
            elif self.drone_state == DroneState.AUTO_DWELLING:
                elapsed_dwell = now - self.dwell_start
                remaining     = DWELL_TIME - elapsed_dwell

                # Log countdown every 5 seconds
                if int(elapsed_dwell) % 5 == 0 and not self.dwell_logged:
                    self.dwell_logged = True
                    self._print_dwell_countdown(remaining)
                elif int(elapsed_dwell) % 5 != 0:
                    self.dwell_logged = False

                if remaining <= 0:
                    # Dwell complete → query LLM for next intent
                    self.get_logger().info(
                        f"[DWELL] ✓ 45s complete at {self.current_wp_name} "
                        f"→ Querying LLM for next intent..."
                    )
                    self.drone_state  = DroneState.AUTO_INIT
                    self.dwell_active = False
                    self.last_llm_time = 0.0
                    self.trigger_llm()

            # Update loiter circle (HOLD_SAFE)
            self.update_loiter()

        # ======================================================
        #  SEND TRAJECTORY TO PX4
        # ======================================================
        sp           = TrajectorySetpoint()
        sp.timestamp = timestamp
        sp.position  = [float(v) for v in self.target_pos]
        sp.yaw       = (self.circle_angle + math.pi / 2.0
                        if self.loitering else 0.0)
        self.traj_pub.publish(sp)

        # ---- Arm & offboard at startup ----
        if self.counter == 100:
            self.send_cmd(176, 1.0, 6.0)
            self.get_logger().info("[SYS] Offboard mode enabled")
        if self.counter == 120:
            self.send_cmd(400, 1.0)
            self.get_logger().info("[SYS] Armed ✓")

        # ---- Status log every 2.5s ----
        if self.counter % 50 == 0:
            llm_tag   = " 🔄[LLM...]"  if self.llm_running  else ""
            dwell_tag = (f" ⏳[DWELL {DWELL_TIME-(now-self.dwell_start):.0f}s]"
                         if self.dwell_active else "")
            batt_tag  = " ⚠LOW"        if self.battery_pct < BATTERY_LOW_THRESH else ""

            intent_str = (self.current_intent.name
                          if self.current_intent else "NONE")

            self.get_logger().info(
                f"[STATUS] State={self.drone_state.value}{llm_tag}{dwell_tag} | "
                f"Intent={intent_str} | "
                f"WP={self.current_wp_name} | "
                f"Pos={[f'{v:.1f}' for v in self.current_pos]} | "
                f"Batt={self.battery_pct:.1f}%{batt_tag} | "
                f"EKF={self.ekf_confidence:.2f}"
            )

        self.counter += 1

    # =========================================================
    #  COMMAND SENDER
    # =========================================================

    def send_cmd(self, command, p1=0.0, p2=0.0):
        msg                  = VehicleCommand()
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        msg.command          = command
        msg.param1           = float(p1)
        msg.param2           = float(p2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
