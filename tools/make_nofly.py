#!/usr/bin/env python3
"""
Generate matched no-fly zones for the study call sets (G1, Ben 24 Oct).

    python3 tools/make_nofly.py            # BASELINE and A-E -> calls/nofly/<SET>.json
    python3 tools/make_nofly.py --check    # print placement checks only

Every set gets the same zone types and sizes at matched positions, defined by
the calls' dispatch rank (see TEMPLATE): three zones lie across the straight
line to or between given calls (most urgent call, middle pair, least urgent
pair), one lies in the area without blocking any home/call line. So zones
sometimes block important calls and sometimes not (Ben, 24 Oct), the same way
in every set. Each zone is moved along its line (then sideways) only as far as
needed to stay clear of home, every call pin and scene, and the other zones.
"blocks" lists every home/call straight line the zone cuts. Output is
deterministic; the files are committed and
read by make_world.py (minimap, hazard monitor) and upload_nofly.py (QGC).
"""
import itertools
import json
import math
import os
import sys

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
sys.path.insert(0, os.path.join(REPO, "ros_ws/src/uav_control/uav_control"))
import call_sets  # noqa: E402

SETS = ["BASELINE", "A", "B", "C", "D", "E"]
# Ben, 24 Oct: zones should sometimes block important calls and sometimes not, and
# be matched across sets. Positions are defined by the calls' DISPATCH rank (the
# urgency the dispatcher gave; 1 = most urgent), not by a route, because the agent
# now plans its own order (agent_planner.py):
#   (label, radius m, kind, a, b, fraction)
#   kind "leg": across the straight line a -> b (0 = home, k = call of dispatch rank k)
#   kind "off": clear of every straight line between home and the calls (no blocking)
TEMPLATE = [
    ("Medevac helicopter landing zone", 90.0, "leg", 0, 1, 0.55),   # blocks the way to the most urgent call
    ("Incident command post", 70.0, "leg", 2, 3, 0.50),             # blocks between the 2nd and 3rd most urgent
    ("Downed power lines", 60.0, "leg", 4, 5, 0.50),                # blocks between the two least urgent
    ("Media / TFR area", 80.0, "off", None, None, None),            # in the area, blocks nothing directly
]
CLEAR_CALL_M = 70.0      # gap between a zone edge and any call pin / scene
CLEAR_HOME_M = 150.0     # gap between a zone edge and home
CLEAR_LINE_M = 40.0      # "off" zone: gap between its edge and every home/call line


def points(set_id):
    """[home, call of rank 1, ..., rank 5] as (east, north), the call ids, and the calls."""
    sc = json.load(open(os.path.join(REPO, "scenario.json")))
    cs, calls = call_sets.resolve(set_id, sc["home"], float(sc.get("cruise_alt_m", 10)))
    by_rank = sorted(calls, key=lambda c: c["priority"])
    pts = [(0.0, 0.0)] + [(c["pos"][1], c["pos"][0]) for c in by_rank]
    names = ["home"] + [c["call_id"] for c in by_rank]
    return pts, names, calls


def scenes(set_id):
    p = os.path.join(REPO, "worlds", f"egle_{set_id}_scenes.json")
    try:
        return [tuple(v["scene_en"]) for v in json.load(open(p)).values()]
    except Exception:
        return []


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy or 1e-9
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def clear(x, y, r, avoid):
    if math.hypot(x, y) < r + CLEAR_HOME_M:
        return False
    return all(math.hypot(x - ax, y - ay) >= r + CLEAR_CALL_M for ax, ay in avoid)


def blocked_lines(z, pts, names):
    out = []
    for i, j in itertools.combinations(range(len(pts)), 2):
        if seg_dist(z["east_m"], z["north_m"], *pts[i], *pts[j]) < z["radius_m"]:
            out.append(f"{names[i]}-{names[j]}")
    return out


def place(set_id):
    pts, names, calls = points(set_id)
    avoid = [(c["pos"][1], c["pos"][0]) for c in calls] + scenes(set_id)
    zones = []
    for k, (label, r, kind, a, b, frac) in enumerate(TEMPLATE):
        others = avoid + [(z["east_m"], z["north_m"]) for z in zones]
        found = None
        if kind == "leg":
            (x0, y0), (x1, y1) = pts[a], pts[b]
            L = math.hypot(x1 - x0, y1 - y0)
            ux, uy = (x1 - x0) / L, (y1 - y0) / L
            cands = []
            for df in [0.0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2, 0.25, -0.25]:
                for side in [0.0, 0.3, -0.3, 0.6, -0.6]:
                    cands.append((abs(df) + abs(side) * 0.5, df, side))
            for _, df, side in sorted(cands):
                f = min(0.9, max(0.1, frac + df))
                cx = x0 + ux * L * f - uy * side * r
                cy = y0 + uy * L * f + ux * side * r
                if clear(cx, cy, r, others) and all(math.hypot(cx - z["east_m"], cy - z["north_m"]) > r + z["radius_m"] + 30
                                                    for z in zones):
                    found = (cx, cy, round(f, 2), side)
                    break
        else:
            lines = list(itertools.combinations(range(len(pts)), 2))
            for rad in (450, 550, 650, 350, 750):
                for ang in range(0, 360, 10):
                    cx, cy = rad * math.sin(math.radians(ang)), rad * math.cos(math.radians(ang))
                    if not clear(cx, cy, r, others):
                        continue
                    if any(math.hypot(cx - z["east_m"], cy - z["north_m"]) < r + z["radius_m"] + 30 for z in zones):
                        continue
                    if all(seg_dist(cx, cy, *pts[i], *pts[j]) >= r + CLEAR_LINE_M for i, j in lines):
                        found = (cx, cy, None, None)
                        break
                if found:
                    break
        if found is None:
            print(f"WARNING {set_id}: no clear spot for zone {k + 1} ({label})")
            continue
        cx, cy, f, side = found
        z = {"id": f"NFZ{k + 1}", "name": label, "type": "circle", "east_m": round(cx, 1),
             "north_m": round(cy, 1), "radius_m": r, "kind": kind,
             "leg": f"{names[a]}->{names[b]}" if kind == "leg" else None,
             "leg_fraction": f, "lateral_offset_r": side}
        z["blocks"] = blocked_lines(z, pts, names)
        zones.append(z)
    return zones


def to_latlon(e, n):
    sc = json.load(open(os.path.join(REPO, "scenario.json")))
    lat0, lon0 = sc["home"]["lat"], sc["home"]["lon"]
    m_lon = call_sets.M_LAT * math.cos(math.radians(lat0))
    return lat0 + n / call_sets.M_LAT, lon0 + e / m_lon


def main():
    out = os.path.join(REPO, "calls", "nofly")
    os.makedirs(out, exist_ok=True)
    for s in SETS:
        zones = place(s)
        for z in zones:
            z["lat"], z["lon"] = [round(v, 6) for v in to_latlon(z["east_m"], z["north_m"])]
        parts = []
        for z in zones:
            where = z["leg"] if z["kind"] == "leg" else "off-route"
            parts.append(f"{z['id']} {where} blocks [{' '.join(z['blocks']) or '-'}]")
        line = "; ".join(parts)
        print(f"{s:9s} {len(zones)} zones: {line}")
        if "--check" not in sys.argv:
            json.dump({"_help": "No-fly zones for this call set (generated by tools/make_nofly.py; matched across sets). "
                                "Condition wears down strongly inside a zone; the agent routes around them.",
                       "set_id": s, "zones": zones}, open(os.path.join(out, f"{s}.json"), "w"), indent=2)


if __name__ == "__main__":
    main()
