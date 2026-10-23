"""
Make PX4's simulated battery (shown in QGroundControl) follow the battery
model, so QGC and the dashboard show the same percentage (D2).

PX4's SITL battery drains to SIM_BAT_MIN_PCT within SIM_BAT_DRAIN seconds and
then stays there; PX4 documents SIM_BAT_MIN_PCT as a way to "alter the battery
level during SITL ... on the fly". This thread sets SIM_BAT_MIN_PCT to the
model's percentage every PERIOD_S over MAVLink, so PX4's value tracks it.
set_px4_params.py sets SIM_BAT_MIN_PCT = 100 at launch so PX4 starts full.
"""
import threading
import time

PERIOD_S = 2.0
MIN_CHANGE_PCT = 0.05


class PX4BatteryDisplay:
    def __init__(self, get_pct, logger=None, url="udp:127.0.0.1:14540"):
        self.get_pct = get_pct
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
        while self.running:
            pct = float(self.get_pct())
            if self.last_sent is None or abs(pct - self.last_sent) >= MIN_CHANGE_PCT:
                try:
                    master.mav.param_set_send(master.target_system, master.target_component,
                                              b"SIM_BAT_MIN_PCT", max(0.0, min(100.0, pct)), real32)
                    self.last_sent = pct
                except Exception as e:
                    self._log(f"[BATT] QGC battery sync send failed: {type(e).__name__}", warn=True)
            time.sleep(PERIOD_S)
