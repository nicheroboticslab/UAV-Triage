"""
Condition (vehicle damage) and hazards (G1/G3/G4, E2; Ben 24 Oct).

Condition starts at 100 % and wears down:
  - impact with a building, tree crown or scene object: a one-off loss that grows
    with impact speed, then a small continuous loss while still in contact;
  - hard ground impact (descending faster than ground_impact_ms);
  - inside a no-fly zone (any altitude): a strong continuous loss.
At 0 % the vehicle is wrecked: the block ends (pilot script lands it in place).

Geometry comes from worlds/egle_<SET>_hazards.json, written by tools/make_world.py
from exactly what the world contains (OSM footprints and heights, tree crowns,
scene boxes, no-fly zones from calls/nofly/<SET>.json). Positions are east/north/up
metres from home, the same frame as the Gazebo world.

Also provides the agent's detour planner around no-fly zones (G3, simple form):
via-points that pass each blocking zone at its radius plus a margin.
All settings: condition.json.
"""
import json
import math
import os

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
DEFAULTS = {
    "impact_base_pct": 5.0,        # loss at the moment of an impact ...
    "impact_per_ms_pct": 2.0,      # ... plus this per m/s of impact speed
    "impact_max_pct": 60.0,
    "scrape_pct_per_s": 2.0,       # while still touching the obstacle
    "ground_impact_ms": 3.0,       # descending faster than this at the ground = impact
    "nofly_pct_per_s": 4.0,        # inside a no-fly zone ("significantly")
    "drone_radius_m": 0.5,
    "agent_nofly_margin_m": 35.0,  # agent detours pass zones at radius + this
}
GRID = 50.0


def load_config(path=None):
    path = path or os.path.join(REPO, "condition.json")
    cfg = dict(DEFAULTS)
    try:
        cfg.update({k: v for k, v in json.load(open(path)).items() if not k.startswith("_")})
    except FileNotFoundError:
        pass
    return cfg


def _pip(x, y, poly):
    inside, j = False, len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy or 1e-9
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class ConditionMonitor:
    def __init__(self, hazards_path, cfg=None):
        self.cfg = cfg or load_config()
        self.condition = 100.0
        self.contacts = set()
        self.in_zone = None
        self.destroyed = False
        self.ok = False
        self.zones, self.buildings, self.trees, self.boxes = [], [], [], []
        self.grid = {}
        try:
            hz = json.load(open(hazards_path))
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
            return
        self.zones = hz.get("zones", [])
        self.trees = hz.get("trees", [])
        self.boxes = hz.get("scene_boxes", [])
        for b in hz.get("buildings", []):
            xs = [p[0] for p in b["pts"]]; ys = [p[1] for p in b["pts"]]
            b["bbox"] = (min(xs), max(xs), min(ys), max(ys))
            self.buildings.append(b)
            for gx in range(int(math.floor(b["bbox"][0] / GRID)), int(math.floor(b["bbox"][1] / GRID)) + 1):
                for gy in range(int(math.floor(b["bbox"][2] / GRID)), int(math.floor(b["bbox"][3] / GRID)) + 1):
                    self.grid.setdefault((gx, gy), []).append(b)
        self.ok = True

    # ---------------- geometry
    def zone_at(self, e, n):
        for z in self.zones:
            if math.hypot(e - z["east_m"], n - z["north_m"]) <= z["radius_m"]:
                return z
        return None

    def touching(self, e, n, u):
        r = self.cfg["drone_radius_m"]
        hits = []
        for b in self.grid.get((int(math.floor(e / GRID)), int(math.floor(n / GRID))), []):
            x0, x1, y0, y1 = b["bbox"]
            if not (x0 - r <= e <= x1 + r and y0 - r <= n <= y1 + r) or u > b["h"] + r:
                continue
            pts = b["pts"]
            if _pip(e, n, pts) or min(_seg_dist(e, n, *pts[i], *pts[(i + 1) % len(pts)])
                                      for i in range(len(pts))) <= r:
                hits.append(f"building {b['id']}")
        for k, t in enumerate(self.trees):
            if abs(e - t["x"]) < t["r"] + r and abs(n - t["y"]) < t["r"] + r:
                if math.sqrt((e - t["x"]) ** 2 + (n - t["y"]) ** 2 + (u - t["zc"]) ** 2) <= t["r"] + r:
                    hits.append(f"tree {k}")
        for bx in self.boxes:
            c, s = math.cos(-bx["yaw"]), math.sin(-bx["yaw"])
            lx = c * (e - bx["x"]) - s * (n - bx["y"])
            ly = s * (e - bx["x"]) + c * (n - bx["y"])
            if (abs(lx) <= bx["sx"] / 2 + r and abs(ly) <= bx["sy"] / 2 + r
                    and abs(u - bx["z"]) <= bx["sz"] / 2 + r):
                hits.append(f"object {bx['name']}")
        return hits

    # ---------------- one control step
    def step(self, e, n, u, vel_enu, dt, flying=True):
        """Returns a list of event dicts (impact, contact_end, nofly_enter, nofly_exit,
        ground_impact, destroyed). self.condition is updated in place."""
        if not self.ok or self.destroyed:
            return []
        c = self.cfg
        speed = math.sqrt(sum(v * v for v in vel_enu))
        events = []
        before = self.condition
        hits = set(self.touching(e, n, u)) if flying else set()
        for h in sorted(hits - self.contacts):
            loss = min(c["impact_max_pct"], c["impact_base_pct"] + c["impact_per_ms_pct"] * speed)
            self.condition -= loss
            events.append({"type": "impact", "object": h, "speed_ms": round(speed, 2), "loss_pct": round(loss, 2)})
        for h in sorted(self.contacts - hits):
            events.append({"type": "contact_end", "object": h})
        if hits & self.contacts:
            self.condition -= c["scrape_pct_per_s"] * dt
        self.contacts = hits
        # hard ground contact
        if flying and u < 0.3 and -vel_enu[2] > c["ground_impact_ms"]:
            if "ground" not in self.contacts:
                loss = min(c["impact_max_pct"], c["impact_base_pct"] + c["impact_per_ms_pct"] * -vel_enu[2])
                self.condition -= loss
                events.append({"type": "ground_impact", "speed_ms": round(-vel_enu[2], 2), "loss_pct": round(loss, 2)})
            self.contacts.add("ground")
        # no-fly zones
        z = self.zone_at(e, n) if flying else None
        zid = z["id"] if z else None
        if zid != (self.in_zone["id"] if self.in_zone else None):
            if self.in_zone:
                events.append({"type": "nofly_exit", "zone": self.in_zone["id"], "name": self.in_zone["name"]})
            if z:
                events.append({"type": "nofly_enter", "zone": z["id"], "name": z["name"]})
            self.in_zone = z
        if z:
            self.condition -= c["nofly_pct_per_s"] * dt
        self.condition = max(0.0, self.condition)
        if self.condition <= 0.0 and before > 0.0:
            self.destroyed = True
            events.append({"type": "destroyed"})
        return events

    # ---------------- agent detours (G3)
    def detour(self, start, goal, n_sides=16):
        """Shortest path from start to goal (east, north) that keeps every no-fly zone
        at radius + margin: visibility graph over a polygon around each inflated zone
        (vertices on the circumscribed n-gon, so straight edges never cut the
        circle), Dijkstra. Returns the via-points (east, north), [] if the straight
        line is clear. A zone that already contains start (or goal) is ignored for
        edges leaving start (entering goal), so the vehicle can always get out."""
        m = self.cfg["agent_nofly_margin_m"]
        zs = [(z["east_m"], z["north_m"], z["radius_m"] + m) for z in self.zones]
        if not zs:
            return []
        inside_s = {i for i, (cx, cy, R) in enumerate(zs) if math.hypot(start[0] - cx, start[1] - cy) < R}
        inside_g = {i for i, (cx, cy, R) in enumerate(zs) if math.hypot(goal[0] - cx, goal[1] - cy) < R}

        def free(a, b, skip=()):
            for i, (cx, cy, R) in enumerate(zs):
                if i in skip:
                    continue
                if _seg_dist(cx, cy, a[0], a[1], b[0], b[1]) < R - 0.5:
                    return False
            return True

        if free(start, goal, inside_s | inside_g):
            return []
        nodes = [tuple(start), tuple(goal)]
        k = 1.0 / math.cos(math.pi / n_sides)
        for i, (cx, cy, R) in enumerate(zs):
            for j in range(n_sides):
                ang = 2 * math.pi * (j + 0.5) / n_sides
                v = (cx + R * k * math.cos(ang), cy + R * k * math.sin(ang))
                if all(math.hypot(v[0] - ox, v[1] - oy) >= oR - 0.5 for ox, oy, oR in zs):
                    nodes.append(v)
        N = len(nodes)
        dist = [math.inf] * N
        prev = [None] * N
        dist[0] = 0.0
        done = [False] * N
        for _ in range(N):
            u = min((d, i) for i, d in enumerate(dist) if not done[i])[1] if any(not x for x in done) else None
            if u is None or dist[u] == math.inf:
                break
            done[u] = True
            if u == 1:
                break
            for v in range(N):
                if done[v] or v == u:
                    continue
                skip = (inside_s if u == 0 else set()) | (inside_g if v == 1 else set())
                w = math.dist(nodes[u], nodes[v])
                if dist[u] + w < dist[v] and free(nodes[u], nodes[v], skip):
                    dist[v] = dist[u] + w
                    prev[v] = u
        if prev[1] is None:
            return []                     # no path found (should not happen): fly straight
        path, i = [], prev[1]
        while i is not None and i != 0:
            path.append(nodes[i])
            i = prev[i]
        return [(round(x, 2), round(y, 2)) for x, y in reversed(path)]
