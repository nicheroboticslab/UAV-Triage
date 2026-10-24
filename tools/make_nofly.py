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
import re
import sys
import xml.etree.ElementTree as ET

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
sys.path.insert(0, os.path.join(REPO, "ros_ws/src/uav_control/uav_control"))
import call_sets  # noqa: E402
import hazards  # noqa: E402
import agent_planner  # noqa: E402
BATT = json.load(open(os.path.join(REPO, "battery.json")))

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
CLEAR_SITE_M = 10.0      # Ben, 24 Oct: zone edge at least 10 m from every call pin, scene anchor
                         # and every person / object actually spawned in the scene
CLEAR_HOME_M = 150.0     # gap between a zone edge and home
CLEAR_LINE_M = 40.0      # "off" zone: gap between its edge and every home/call line
CLEAR_ZONE_M = 30.0      # gap between zones
TARGET_TOUR_M = None     # None: common target chosen from what every set can reach (see balance())
TOL = 0.01               # balanced when each set's agent tour is within +-1 % of the target


def points(set_id):
    """[home, call of rank 1, ..., rank 5] as (east, north), the call ids, and the calls."""
    sc = json.load(open(os.path.join(REPO, "scenario.json")))
    cs, calls = call_sets.resolve(set_id, sc["home"], float(sc.get("cruise_alt_m", 10)))
    by_rank = sorted(calls, key=lambda c: c["priority"])
    pts = [(0.0, 0.0)] + [(c["pos"][1], c["pos"][0]) for c in by_rank]
    names = ["home"] + [c["call_id"] for c in by_rank]
    return pts, names, calls


def site_points(set_id, calls):
    """Call pins, scene anchors and every scene object/person position in the built world."""
    out = [(c["pos"][1], c["pos"][0]) for c in calls]
    try:
        out += [tuple(v["scene_en"]) for v in json.load(open(os.path.join(REPO, "worlds", f"egle_{set_id}_scenes.json"))).values()]
    except Exception:
        print(f"WARNING {set_id}: no worlds/egle_{set_id}_scenes.json (build the world first)")
    sdf = os.path.join(REPO, "worlds", f"egle_{set_id}.sdf")
    n_obj = 0
    try:
        root = ET.parse(sdf).getroot()
        for el in root.iter():
            name = el.get("name") or (el.findtext("name") if el.tag == "include" else None) or ""
            if el.tag not in ("model", "actor", "include") or not re.match(r"c\d_", name):
                continue
            pose = el.find("pose") if el.tag != "actor" else el.find(".//waypoint/pose")
            if pose is None:
                continue
            x, y = [float(v) for v in pose.text.split()[:2]]
            out.append((x, y))
            n_obj += 1
    except Exception as e:
        print(f"WARNING {set_id}: could not read {sdf} ({e}); only pins and anchors are checked")
    return out, n_obj


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy or 1e-9
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def blocked_lines(z, pts, names):
    out = []
    for i, j in itertools.combinations(range(len(pts)), 2):
        if seg_dist(z["east_m"], z["north_m"], *pts[i], *pts[j]) < z["radius_m"]:
            out.append(f"{names[i]}-{names[j]}")
    return out


def candidates(k, pts, sites):
    """All valid positions for template zone k (ignoring the other zones)."""
    label, r, kind, a, b, _ = TEMPLATE[k]
    out = []

    def ok(cx, cy):
        return (math.hypot(cx, cy) >= r + CLEAR_HOME_M
                and all(math.hypot(cx - sx, cy - sy) >= r + CLEAR_SITE_M for sx, sy in sites))
    if kind == "leg":
        (x0, y0), (x1, y1) = pts[a], pts[b]
        L = math.hypot(x1 - x0, y1 - y0)
        ux, uy = (x1 - x0) / L, (y1 - y0) / L
        for i in range(3, 18):                       # fraction 0.15 .. 0.85
            f = i * 0.05
            for side in (-0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75):   # |offset| < r: still cuts the line
                cx, cy = x0 + ux * L * f - uy * side * r, y0 + uy * L * f + ux * side * r
                if ok(cx, cy):
                    out.append((cx, cy, round(f, 2), side))
    else:
        lines = list(itertools.combinations(range(len(pts)), 2))
        for rad in range(350, 851, 50):
            for ang in range(0, 360, 10):
                cx, cy = rad * math.sin(math.radians(ang)), rad * math.cos(math.radians(ang))
                if ok(cx, cy) and all(seg_dist(cx, cy, *pts[i], *pts[j]) >= r + CLEAR_LINE_M for i, j in lines):
                    out.append((cx, cy, None, None))
    return out


def zone(k, c, names):
    label, r, kind, a, b, _ = TEMPLATE[k]
    return {"id": f"NFZ{k + 1}", "name": label, "type": "circle", "east_m": round(c[0], 1), "north_m": round(c[1], 1),
            "radius_m": r, "kind": kind, "leg": f"{names[a]}->{names[b]}" if kind == "leg" else None,
            "leg_fraction": c[2], "lateral_offset_r": c[3]}


def apart(zs):
    return all(math.hypot(p["east_m"] - q["east_m"], p["north_m"] - q["north_m"]) >= p["radius_m"] + q["radius_m"] + CLEAR_ZONE_M
               for p, q in itertools.combinations(zs, 2))


def agent_tour(zones, calls):
    """Length (m) and order of the agent's mission with these zones: the planner
    (agent_planner.py) re-plans after every call, flies the no-fly-aware path,
    holds hold_est_s per call, then returns home - as in the pilot, but with no
    battery limit (see below)."""
    cm = hazards.ConditionMonitor.__new__(hazards.ConditionMonitor)
    cm.zones, cm.ok, cm.cfg = zones, True, hazards.load_config()
    pl = agent_planner.Planner(cm, BATT, agent_planner.load_config())
    pc = [{"id": c["call_id"], "e": c["pos"][1], "n": c["pos"][0], "rank": c["priority"], "done": False} for c in calls]
    # Battery is not a constraint here (pct very large): sets are balanced on the full
    # 5-call mission, so a set is not "shorter" just because the agent had to drop a call.
    here, pct, t, total, order = (0.0, 0.0), 1e6, 0.0, 0.0, []
    while True:
        plan = pl.plan(here, pct, pc, t)
        if not plan["order"]:
            break
        nxt = next(x for x in pc if x["id"] == plan["order"][0])
        d = pl.length(here, (nxt["e"], nxt["n"]))
        total += d
        t += d / pl.cfg["cruise_ms"] + pl.cfg["hold_est_s"]
        pct = pl.energy.after_hover(pl.energy.after_flight(pct, d), pl.cfg["hold_est_s"])
        here = (nxt["e"], nxt["n"])
        nxt["done"] = True
        order.append(nxt["id"])
    total += pl.length(here, (0.0, 0.0))
    return total, order


def optimal_tour(zones, calls):
    """Shortest closed tour home -> all calls -> home along no-fly-aware paths (the
    set-matching criterion of decisions log §6.2, now including the detours)."""
    cm = hazards.ConditionMonitor.__new__(hazards.ConditionMonitor)
    cm.zones, cm.ok, cm.cfg = zones, True, hazards.load_config()
    pl = agent_planner.Planner(cm, BATT, agent_planner.load_config())
    pts = [(c["pos"][1], c["pos"][0]) for c in calls]
    best = None
    for perm in itertools.permutations(range(len(pts))):
        path = [(0.0, 0.0)] + [pts[i] for i in perm] + [(0.0, 0.0)]
        L = sum(pl.length(path[i], path[i + 1]) for i in range(len(path) - 1))
        if best is None or L < best[0]:
            best = (L, [calls[i]["call_id"] for i in perm])
    return best


METRIC = optimal_tour


def descend(cands, calls, names, cost, start=None, rounds=3):
    """Coordinate descent over the zones' candidate positions minimising cost(length)."""
    if start is None:                     # greedy feasible start: middle-most candidate apart from earlier zones
        cur, zs = [], []
        for k, ck in enumerate(cands):
            order = sorted(range(len(ck)), key=lambda i: abs(i - len(ck) // 2))
            pick = next((ck[i] for i in order if apart(zs + [zone(k, ck[i], names)])), None)
            if pick is None:
                raise RuntimeError(f"zone {k + 1}: no position apart from the others")
            cur.append(pick)
            zs.append(zone(k, pick, names))
    else:
        cur = list(start)
        zs = [zone(k, cur[k], names) for k in range(len(cur))]
    best_L, _ = METRIC(zs, calls)
    best = cost(best_L)
    for _ in range(rounds):
        improved = False
        for k in range(len(cur)):
            for c in cands[k][::2] if len(cands[k]) > 40 else cands[k]:
                trial = zs[:k] + [zone(k, c, names)] + zs[k + 1:]
                if not apart(trial):
                    continue
                L, _ = METRIC(trial, calls)
                if cost(L) < best - 1e-6:
                    best, best_L, cur, zs, improved = cost(L), L, cur[:k] + [c] + cur[k + 1:], trial, True
        if not improved:
            break
    return zs, best_L


def balance():
    """Pick zone positions (within the template) so every set's agent tour has the
    same length: find each set's reachable range, take a common target inside all of
    them (closest to the mean of the midpoints), then descend toward it."""
    data = {}
    for s in SETS:
        pts, names, calls = points(s)
        sites, n_obj = site_points(s, calls)
        cands = [candidates(k, pts, sites) for k in range(len(TEMPLATE))]
        for k, c in enumerate(cands):
            if not c:
                sys.exit(f"{s}: no valid position for zone {k + 1} ({TEMPLATE[k][0]})")
        lo, _ = descend(cands, calls, names, lambda L: L, rounds=2)
        hi, _ = descend(cands, calls, names, lambda L: -L, rounds=2)
        lmin, lmax = METRIC(lo, calls)[0], METRIC(hi, calls)[0]
        data[s] = dict(pts=pts, names=names, calls=calls, cands=cands, lmin=lmin, lmax=lmax, n_obj=n_obj, sites=len(sites))
        print(f"{s:9s} reachable optimal tour {lmin / 1000:.2f}-{lmax / 1000:.2f} km "
              f"({len(sites)} site points checked, {n_obj} scene objects)", flush=True)
    lo = max(d["lmin"] for d in data.values())
    hi = min(d["lmax"] for d in data.values())
    mid = sum((d["lmin"] + d["lmax"]) / 2 for d in data.values()) / len(data)
    target = TARGET_TOUR_M or (min(max(mid, lo), hi) if lo <= hi else (lo + hi) / 2)
    print(f"target {target / 1000:.3f} km (common range {lo / 1000:.2f}-{hi / 1000:.2f} km)", flush=True)
    out = {}
    for s, d in data.items():
        zs, L = descend(d["cands"], d["calls"], d["names"], lambda L: abs(L - target), rounds=4)
        L, order = METRIC(zs, d["calls"])
        out_agent = agent_tour(zs, d["calls"])
        for z in zs:
            z["blocks"] = blocked_lines(z, d["pts"], d["names"])
        out[s] = (zs, L, order, target, out_agent)
    return out


def to_latlon(e, n):
    sc = json.load(open(os.path.join(REPO, "scenario.json")))
    lat0, lon0 = sc["home"]["lat"], sc["home"]["lon"]
    m_lon = call_sets.M_LAT * math.cos(math.radians(lat0))
    return lat0 + n / call_sets.M_LAT, lon0 + e / m_lon


def main():
    out = os.path.join(REPO, "calls", "nofly")
    os.makedirs(out, exist_ok=True)
    res = balance()
    for s, (zones, L, order, target, agent) in res.items():
        for z in zones:
            z["lat"], z["lon"] = [round(v, 6) for v in to_latlon(z["east_m"], z["north_m"])]
        parts = [f"{z['id']} {z['leg'] or 'off-route'} blocks [{' '.join(z['blocks']) or '-'}]" for z in zones]
        flag = "" if abs(L - target) <= TOL * target else "  (OUTSIDE +-1 %)"
        print(f"{s:9s} optimal tour {L / 1000:.3f} km ({(L - target) / target * 100:+.1f} %) order {' '.join(order)}{flag}"
              f" | agent (no battery limit) {agent[0] / 1000:.2f} km {' '.join(agent[1])}")
        print("          " + "; ".join(parts))
        if "--check" not in sys.argv:
            json.dump({"_help": "No-fly zones for this call set (generated by tools/make_nofly.py; matched across sets "
                                "by dispatch rank, positioned so the shortest tour around the zones has the same length in "
                                "every set). Condition wears down strongly inside a zone; the agent routes around them.",
                       "set_id": s, "optimal_tour_m": round(L, 1), "optimal_order": order, "target_tour_m": round(target, 1),
                       "agent_tour_m": round(agent[0], 1), "agent_order": agent[1],
                       "zones": zones}, open(os.path.join(out, f"{s}.json"), "w"), indent=2)


if __name__ == "__main__":
    main()
