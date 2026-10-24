"""
No-fly zone monitor for the UAV-Triage pilot scripts.

Reads no_fly_zones from scenario.json, checks the drone's local position
every loop cycle, and records each ENTER / EXIT event:
  - to the flight log (Dashboard) and ROS console
  - to ~/uav_dataset/nofly_<timestamp>.jsonl  (one JSON object per event)

Zones cover all altitudes. Position is local NED metres from home, the same
frame as MAP and self.current_pos.
"""
import json
import math
import os
import time
from datetime import datetime


def _point_in_polygon(x, y, poly):
    """Ray casting; poly is a list of (x, y)."""
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


class NoFlyMonitor:
    def __init__(self, dataset_dir, logger=None, log_event=None, scenario_path=None):
        self.logger = logger
        self.log_event = log_event
        path = scenario_path or os.environ.get(
            "UAV_SCENARIO", os.path.expanduser("~/UAV-Triage/scenario.json"))
        with open(path) as f:
            sc = json.load(f)
        hlat, hlon = sc["home"]["lat"], sc["home"]["lon"]
        m_lat = 111320.0
        m_lon = 111320.0 * math.cos(math.radians(hlat))

        def ned(lat, lon):
            return ((lat - hlat) * m_lat, (lon - hlon) * m_lon)

        zones = sc.get("no_fly_zones", {})
        self.zones = []
        for c in zones.get("circles", []):
            n, e = ned(c["lat"], c["lon"])
            self.zones.append({"name": c["name"], "type": "circle",
                               "center": (n, e), "radius": float(c["radius_m"])})
        for p in zones.get("polygons", []):
            self.zones.append({"name": p["name"], "type": "polygon",
                               "points": [ned(lat, lon) for lat, lon in p["points"]]})

        self.inside = {}        # zone name -> entry info
        self.total_entries = 0
        self.total_seconds = 0.0

        os.makedirs(dataset_dir, exist_ok=True)
        self.out_path = os.path.join(
            dataset_dir, f"nofly_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl")
        self._say(f"[NO-FLY] Monitoring {len(self.zones)} zone(s). Events -> {self.out_path}")

    def _say(self, msg, warn=False):
        if self.logger:
            # rclpy raises if one source line logs at two severities, so keep
            # the WARN and INFO calls on separate lines.
            if warn:
                self.logger.warn(msg)
            else:
                self.logger.info(msg)
        if self.log_event:
            try:
                self.log_event(msg)
            except Exception:
                pass

    def _contains(self, z, n, e):
        if z["type"] == "circle":
            return math.hypot(n - z["center"][0], e - z["center"][1]) <= z["radius"]
        return _point_in_polygon(n, e, z["points"])

    def _write(self, rec):
        with open(self.out_path, "a") as f:
            f.write(json.dumps(rec) + "\n")

    def update(self, pos, state, t_mission, waypoint=None):
        """Call every loop cycle. pos = [north, east, down] metres."""
        n, e = float(pos[0]), float(pos[1])
        now = time.time()
        state_name = getattr(state, "name", str(state))
        for z in self.zones:
            is_in = self._contains(z, n, e)
            was_in = z["name"] in self.inside
            if is_in and not was_in:
                self.inside[z["name"]] = {"t": t_mission, "wall": now, "state": state_name}
                self.total_entries += 1
                self._write({"event": "ENTER", "zone": z["name"], "t_mission_s": round(t_mission, 2),
                             "wall_time": now, "pos_ned": [round(n, 1), round(e, 1)],
                             "altitude_m": round(-float(pos[2]), 1), "control_state": state_name,
                             "heading_to": waypoint})
                self._say(f"[NO-FLY] ENTERED '{z['name']}' — control: {state_name}", warn=True)
            elif was_in and not is_in:
                info = self.inside.pop(z["name"])
                dur = t_mission - info["t"]
                self.total_seconds += dur
                self._write({"event": "EXIT", "zone": z["name"], "t_mission_s": round(t_mission, 2),
                             "wall_time": now, "pos_ned": [round(n, 1), round(e, 1)],
                             "duration_s": round(dur, 2), "control_state": state_name,
                             "control_state_at_entry": info["state"], "heading_to": waypoint})
                self._say(f"[NO-FLY] EXITED '{z['name']}' after {dur:.1f}s — control: {state_name}")

    def summary(self):
        return (f"No-fly: {self.total_entries} entries, {self.total_seconds:.1f}s inside "
                f"(completed), currently inside: {list(self.inside) or 'none'}")
