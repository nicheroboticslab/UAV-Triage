"""
Make PX4's simulated battery (shown in QGroundControl) follow the battery
model, so QGC and the dashboard show the same percentage (D2).

PX4's SITL battery drains to SIM_BAT_MIN_PCT within SIM_BAT_DRAIN seconds and
then stays there; PX4 documents SIM_BAT_MIN_PCT as a way to "alter the battery
level during SITL ... on the fly". This thread sets SIM_BAT_MIN_PCT to the
model's percentage every PERIOD_S over MAVLink, so PX4's value tracks it.
set_px4_params.py sets SIM_BAT_MIN_PCT = 100 at launch so PX4 starts full.

Camera gimbal (E3): the same MAVLink link sends the operator's gimbal angles
(get_gimbal() -> (pitch_down_deg, yaw_right_deg), relative to the nose) as
MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW at up to GIMBAL_HZ. PX4's gimbal module
stabilises the camera (MNT_* params from airframe 4019_gz_x500_gimbal). Once at
connect it sends MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE to take primary control.
https://mavlink.io/en/services/gimbal_v2.html
"""
import math
import threading
import time

PERIOD_S = 2.0          # battery sync period
GIMBAL_HZ = 10.0
GIMBAL_RESEND_S = 1.0   # resend unchanged angles this often
MIN_CHANGE_PCT = 0.05


class PX4BatteryDisplay:
    def __init__(self, get_pct, logger=None, url="udp:127.0.0.1:14540", get_gimbal=None):
        self.get_pct = get_pct
        self.get_gimbal = get_gimbal
        self.logger = logger
        self.url = url
        self.running = True
        self.last_sent = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _log(self, msg, warn=False):
        if self.logger is None:
            return
        if warn:
            self.logger.warn(msg)
        else:
            self.logger.info(msg)

    def start(self):
        self._thread.start()

    def stop(self):
        self.running = False

    def _run(self):
        try:
            from pymavlink import mavutil
            master = mavutil.mavlink_connection(self.url)
            if master.wait_heartbeat(timeout=15) is None:
                self._log("[BATT] QGC battery sync: no MAVLink heartbeat; QGC will show PX4's own battery", warn=True)
                return
        except Exception as e:
            self._log(f"[BATT] QGC battery sync disabled: {type(e).__name__}: {e}", warn=True)
            return
        self._log(f"[BATT] QGC battery sync active ({self.url})")
        real32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        if self.get_gimbal is not None:
            self._gimbal_configure(master, mavutil)
        next_batt = 0.0
        while self.running:
            now = time.time()
            if self.get_gimbal is not None:
                self._gimbal_tick(master, mavutil, now)
            if now < next_batt:
                time.sleep(1.0 / GIMBAL_HZ)
                continue
            next_batt = now + PERIOD_S
            pct = float(self.get_pct())
            if self.last_sent is None or abs(pct - self.last_sent) >= MIN_CHANGE_PCT:
                try:
                    master.mav.param_set_send(master.target_system, master.target_component,
                                              b"SIM_BAT_MIN_PCT", max(0.0, min(100.0, pct)), real32)
                    self.last_sent = pct
                except Exception as e:
                    self._log(f"[BATT] QGC battery sync send failed: {type(e).__name__}", warn=True)

    # ---------- gimbal ----------
    def _gimbal_configure(self, master, mavutil):
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, 0,
                master.mav.srcSystem, master.mav.srcComponent,   # primary control = this link
                -1, -1, 0, 0, 0)                                  # secondary unchanged; all gimbals
            self._log("[GIMBAL] requested primary control")
        except Exception as e:
            self._log(f"[GIMBAL] configure failed: {type(e).__name__}: {e}", warn=True)
        self._g_last = None
        self._g_sent_t = 0.0
        self._g_acks = set()

    def _gimbal_tick(self, master, mavutil, now):
        # Log the first ACK result for each gimbal command (diagnostic)
        try:
            while True:
                ack = master.recv_match(type="COMMAND_ACK", blocking=False)
                if ack is None:
                    break
                key = (ack.command, ack.result)
                if ack.command in (1000, 1001) and key not in self._g_acks:
                    self._g_acks.add(key)
                    self._log(f"[GIMBAL] ack command {ack.command} result {ack.result} (0 = accepted)")
        except Exception:
            pass
        try:
            pitch_down, yaw_right = self.get_gimbal()
        except Exception:
            return
        cur = (round(pitch_down, 1), round(yaw_right, 1))
        if cur == self._g_last and now - self._g_sent_t < GIMBAL_RESEND_S:
            return
        try:
            nan = float("nan")
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, 0,
                -float(pitch_down), float(yaw_right),   # MAVLink: pitch + = up, yaw + = right
                nan, nan,                               # no rate control
                0,                                      # flags 0: yaw follows the vehicle
                0, 0)                                   # gimbal device id 0 = all
            self._g_last, self._g_sent_t = cur, now
        except Exception as e:
            self._log(f"[GIMBAL] send failed: {type(e).__name__}", warn=True)
