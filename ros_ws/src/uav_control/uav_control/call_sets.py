"""
Call-location pool and call-set files (requirements C3, C4, H3).

Files (repo root):
  calls/pool.json           pool of 30-40 call locations (C3)
  calls/sets/<SET>.json     one predetermined call set: up to 5 calls (C4)
session.json  block.call_set = "<SET>"  selects the set for the block;
null keeps the legacy scenario.json sectors.

A call set's calls replace MAP["sectors"] in the pilot script, so the agent's
existing SEARCH_NEXT_PRIORITY logic visits calls by priority.
"""
import itertools
import json
import math
import os

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
POOL_PATH = os.path.join(REPO, "calls", "pool.json")
SET_DIR = os.path.join(REPO, "calls", "sets")
MAX_CALLS = 5
M_LAT = 111320.0


def ned(lat, lon, home, alt_m):
    m_lon = M_LAT * math.cos(math.radians(home["lat"]))
    return [round((lat - home["lat"]) * M_LAT, 1), round((lon - home["lon"]) * m_lon, 1), -float(alt_m)]


def load_pool(path=POOL_PATH):
    with open(path) as f:
        pool = json.load(f)
    return {loc["id"]: loc for loc in pool["locations"]}


def load_set(set_id, set_dir=SET_DIR):
    path = set_id if set_id.endswith(".json") else os.path.join(set_dir, f"{set_id}.json")
    with open(path) as f:
        return json.load(f), path


def resolve(set_id, home, cruise_alt_m=10.0, pool_path=POOL_PATH, set_dir=SET_DIR):
    """Return (call set dict, list of resolved calls with local NED positions).
    Raises ValueError for anything that would make a block invalid."""
    cs, path = load_set(set_id, set_dir)
    pool = load_pool(pool_path)
    calls = cs.get("calls", [])
    if not 1 <= len(calls) <= MAX_CALLS:
        raise ValueError(f"{path}: {len(calls)} calls (must be 1-{MAX_CALLS})")
    out, seen = [], set()
    for c in calls:
        loc_id = c["location"]
        if loc_id not in pool:
            raise ValueError(f"{path}: location {loc_id} not in pool")
        if loc_id in seen:
            raise ValueError(f"{path}: location {loc_id} used twice")
        seen.add(loc_id)
        loc = pool[loc_id]
        if loc.get("lat") is None or loc.get("lon") is None:
            raise ValueError(f"pool.json: {loc_id} ({loc.get('landmark')}) has no coordinates yet")
        alt = float(loc.get("alt_m", cruise_alt_m))
        out.append({
            "call_id": c["call_id"], "location": loc_id, "priority": int(c["priority"]),
            "label": loc.get("label", loc_id), "call_type": c.get("call_type", loc.get("call_type")),
            "scenario_code": c.get("scenario_code"), "condition": c.get("condition", {}),
            "placeholder": bool(loc.get("placeholder", False)),
            "lat": loc["lat"], "lon": loc["lon"], "pos": ned(loc["lat"], loc["lon"], home, alt),
        })
    prios = [c["priority"] for c in out]
    if len(set(prios)) != len(prios):
        raise ValueError(f"{path}: duplicate priorities")
    return cs, out


def as_sectors(calls):
    """Calls in the pilot script's MAP['sectors'] format."""
    return [{"id": i, "name": c["call_id"], "priority": c["priority"], "pos": c["pos"], "label": c["label"],
             "scenario_code": c["scenario_code"]}
            for i, c in enumerate(sorted(calls, key=lambda c: c["priority"]))]


def route_metrics(calls, cruise_ms=14.0, dwell_s=45.0, power=None):
    """H3 matching metrics: optimal closed tour home -> all calls -> home
    (exact, <= 5 calls), its length and time at cruise plus dwell per call,
    and the energy if a battery_model.PowerModel is given."""
    pts = [(c["pos"][0], c["pos"][1]) for c in calls]
    best = None
    for order in itertools.permutations(range(len(pts))):
        path = [(0.0, 0.0)] + [pts[i] for i in order] + [(0.0, 0.0)]
        d = sum(math.dist(path[k], path[k + 1]) for k in range(len(path) - 1))
        if best is None or d < best[0]:
            best = (d, order)
    d, order = best
    transit_s = d / cruise_ms
    total_s = transit_s + dwell_s * len(calls)
    m = {
        "optimal_order": [calls[i]["call_id"] for i in order],
        "route_m": round(d, 1),
        "transit_s": round(transit_s, 1),
        "total_s": round(total_s, 1),
        "radius_m": [round(math.hypot(*p), 1) for p in pts],
        "call_types": sorted(c["call_type"] or "?" for c in calls),
    }
    if power is not None:
        m["energy_wh"] = round((power.level(cruise_ms) * transit_s + power.level(0.0) * dwell_s * len(calls)) / 3600, 2)
    return m
