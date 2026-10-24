#!/usr/bin/env python3
"""
Grade one run's reports and secondary-task answers (J1 components, E4, H5).

    python3 tools/grade_reports.py 20261024_093012            # ~/uav_dataset/runs/<id>/ or ~/uav_dataset
    python3 tools/grade_reports.py ~/uav_dataset/runs/20261024_093012
    python3 tools/grade_reports.py --all                      # every archived run -> grades_all.csv

Writes grades_<run>.json next to the logs and prints a summary.

Reports: the LAST submission per call is scored field by field against the
call set's ground truth (never the call text; decisions log §7.1, §7.5):
  priority      truth.priority
  people        truth.people (5 or more -> "5+")
  water_depth   truth.water_depth mapped to the portal scale
                (None->None, Below knee->Low, Knee to waist->Medium, Chest or deeper->Deep);
                only when truth.depth_scored (people present)
  water_motion  truth.water_motion
  worst         call condition.worst; only when people are present
Calls with no submission are "missed". Revisions and times are kept.

Secondary tasks: graded from the vehicle state logged with each answer (see
tools/make_tasks.py for the definitions); borderline cases (within a stated
tolerance of a boundary) accept either neighbouring answer and are flagged.

Composite score weights are still open (decisions log §8.2), so only the
components are reported; weights can be applied to the JSON later.
"""
import csv
import glob
import json
import math
import os
import sys

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
DATA = os.path.expanduser(os.environ.get("UAV_DATASET", "~/uav_dataset"))
DEPTH_MAP = {"None": "None", "Below knee": "Low", "Knee to waist": "Medium", "Chest or deeper": "Deep"}
COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
TOL = {"status_pct": 3.0, "nearest_m": 25.0, "bearing_deg": 10.0, "nofly_m": 15.0, "distance_m": 30.0}


def jl(path):
    out = []
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass
    except FileNotFoundError:
        pass
    return out


def find_files(arg):
    if os.path.isdir(arg):
        d = arg
        rid = os.path.basename(os.path.normpath(arg))
    else:
        rid = arg
        d = os.path.join(DATA, "runs", rid)
        if not os.path.isdir(d):
            d = DATA
    pick = lambda pre: (sorted(glob.glob(os.path.join(d, f"{pre}_{rid}.json*"))) or [None])[0]
    return rid, d, {k: pick(k) for k in ("reports", "tasks", "meta", "condition", "inaction")}


def truth_for(meta_path, set_id):
    cs = None
    if meta_path:
        m = json.load(open(meta_path))
        cs = (m.get("call_set") or {}).get("content")
    if cs is None and set_id:
        cs = json.load(open(os.path.join(REPO, "calls", "sets", f"{set_id}.json")))
    return {c["call_id"]: c for c in (cs or {}).get("calls", [])}


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy or 1e-9
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def grade_reports(reps, truth, calls_cfg):
    last, first_t, n_sub = {}, {}, {}
    for r in reps:
        if r.get("type") == "submit":
            c = r["call"]
            last[c] = r
            first_t.setdefault(c, r.get("t_s"))
            n_sub[c] = n_sub.get(c, 0) + 1
    out = []
    for cid in sorted(truth):
        c = truth[cid]
        t = c["truth"]
        ppl = t["people"]
        key = {"priority": t["priority"], "people": "5+" if ppl >= 5 else str(ppl),
               "water_motion": t["water_motion"]}
        if t.get("depth_scored", ppl > 0):
            key["water_depth"] = DEPTH_MAP.get(t["water_depth"], t["water_depth"])
        if ppl > 0:
            key["worst"] = c.get("condition", {}).get("worst")
        row = {"call": cid, "location": c["location"], "truth_priority": t["priority"], "key": key,
               "reported": cid in last, "submissions": n_sub.get(cid, 0),
               "first_submit_s": first_t.get(cid), "last_submit_s": last[cid].get("t_s") if cid in last else None}
        if cid in last:
            ans = last[cid]["fields"]
            row["answer"] = ans
            row["fields"] = {k: ans.get(k) == v for k, v in key.items()}
            row["fields_correct"] = sum(row["fields"].values())
            row["fields_scored"] = len(key)
        out.append(row)
    return out


def bearing_to_home(n, e):
    return math.degrees(math.atan2(-e, -n)) % 360.0          # 0 = north, clockwise


def grade_tasks(trec, calls_cfg, zones, truth, dist_bands):
    tasks = {}
    for r in trec:
        if r.get("type") == "task_list":
            tasks = {t["id"]: dict(t, status="not_logged") for t in r["tasks"]}
    calls = {c["id"]: c for c in calls_cfg}
    for r in trec:
        tid = r.get("task")
        if tid not in tasks:
            continue
        t = tasks[tid]
        typ = r["type"]
        if typ == "shown":
            t["status"], t["shown_s"] = "shown", r.get("t_s")
        elif typ == "opened":
            t.setdefault("opened_s", r.get("t_s"))
        elif typ in ("expired", "not_shown"):
            t["status"] = typ
            t["status_reason"] = r.get("reason")
        elif typ == "answer":
            t["status"], t["answer_s"], t["answer"] = "answered", r.get("t_s"), r["values"]
            t["response_s"] = round(r["t_s"] - t["shown_s"], 3) if t.get("shown_s") is not None else None
            t.update(_key(t, r, calls, zones, truth, dist_bands))
    return list(tasks.values())


def _key(t, r, calls, zones, truth, dist_bands):
    snap = r.get("snapshot") or {}
    pos = snap.get("pos_ned") or [0.0, 0.0, 0.0]
    n, e = pos[0], pos[1]
    v = r["values"]
    k = t["kind"]
    if k == "ACK":
        return {"correct": True}
    if k == "STATUS":
        okb = abs(float(v["battery_pct"]) - float(snap.get("battery_pct") or 0)) <= TOL["status_pct"]
        okc = (snap.get("condition_pct") is None
               or abs(float(v["condition_pct"]) - float(snap["condition_pct"])) <= TOL["status_pct"])
        return {"key": {"battery_pct": snap.get("battery_pct"), "condition_pct": snap.get("condition_pct")},
                "parts": {"battery_pct": okb, "condition_pct": okc}, "correct": okb and okc}
    if k == "NEAREST":
        d = {cid: math.hypot(c["n"] - n, c["e"] - e) for cid, c in calls.items()}
        best = min(d.values())
        ok_set = sorted(cid for cid, x in d.items() if x - best <= TOL["nearest_m"])
        return {"key": ok_set, "correct": v["call"] in ok_set, "borderline": len(ok_set) > 1}
    if k == "HOME_BEARING":
        b = bearing_to_home(n, e)
        main = COMPASS[int(round(b / 45.0)) % 8]
        ok = {main}
        off = (b / 45.0) % 1.0                             # distance to the sector edges (edges at x.5)
        edge_deg = abs(off - 0.5) * 45.0
        if edge_deg <= TOL["bearing_deg"]:
            ok.add(COMPASS[int(math.floor(b / 45.0)) % 8]); ok.add(COMPASS[int(math.ceil(b / 45.0)) % 8])
        return {"key": sorted(ok), "bearing_deg": round(b, 1), "correct": v["bearing"] in ok, "borderline": len(ok) > 1}
    if k == "NOFLY_LINE":
        c = calls[t["call"]]
        gaps = [seg_dist(z["east_m"], z["north_m"], e, n, c["e"], c["n"]) - z["radius_m"] for z in zones]
        g = min(gaps) if gaps else 1e9
        key = "Yes" if g < 0 else "No"
        ok = {"Yes", "No"} if abs(g) <= TOL["nofly_m"] else {key}
        return {"key": sorted(ok), "clearance_m": round(g, 1), "correct": v["nofly"] in ok, "borderline": len(ok) > 1}
    if k == "DISTANCE":
        c = calls[t["call"]]
        d = math.hypot(c["n"], c["e"])
        ok = {b[2] for b in dist_bands if b[0] - TOL["distance_m"] <= d < b[1] + TOL["distance_m"]}
        return {"key": sorted(ok), "distance_m": round(d, 1), "correct": v["band"] in ok, "borderline": len(ok) > 1}
    if k == "BOAT":
        reps = r.get("reports_so_far") or {}
        rank = {"P1": 1, "P2": 2}
        def best(prio_of):
            cand = {cid: rank[p] for cid, p in ((cid, prio_of(cid)) for cid in reps) if p in rank}
            if not cand:
                return ["No call needs a boat"]
            m = min(cand.values())
            return sorted(cid for cid, x in cand.items() if x == m)
        k_truth = best(lambda cid: truth[cid]["truth"]["priority"] if cid in truth else None)
        k_self = best(lambda cid: reps[cid].get("priority"))
        return {"key": k_truth, "key_own_reports": k_self, "correct": v["call"] in k_truth,
                "consistent_with_own_reports": v["call"] in k_self}
    return {"correct": None}


def condition_summary(rows):
    s = {"final_pct": None, "impacts": 0, "ground_impacts": 0, "nofly_entries": 0, "destroyed": False, "nofly_s": 0.0}
    last_in = None
    for r in rows:
        t = r.get("type")
        if "condition_pct" in r and t != "config":
            s["final_pct"] = r["condition_pct"]
        if t == "impact":
            s["impacts"] += 1
        elif t == "ground_impact":
            s["ground_impacts"] += 1
        elif t == "nofly_enter":
            s["nofly_entries"] += 1
            last_in = r.get("t_s")
        elif t == "nofly_exit" and last_in is not None:
            s["nofly_s"] += r.get("t_s", last_in) - last_in
            last_in = None
        elif t == "destroyed":
            s["destroyed"] = True
    s["nofly_s"] = round(s["nofly_s"], 1)
    return s


def grade(arg):
    rid, d, f = find_files(arg)
    reps, trec = jl(f["reports"]), jl(f["tasks"])
    cfg = next((r for r in reps + trec if r.get("type") == "config"), None)
    if cfg is None:
        print(f"{rid}: no portal logs")
        return None
    truth = truth_for(f["meta"], cfg.get("set_id"))
    try:
        bands = json.load(open(os.path.join(REPO, "calls", "tasks", f"{cfg['set_id']}.json")))["dist_bands"]
    except Exception:
        bands = [[0, 300, "Under 300 m"], [300, 600, "300-600 m"], [600, 900, "600-900 m"], [900, 99999, "Over 900 m"]]
    out = {"run_id": rid, "set_id": cfg.get("set_id"), "session": cfg.get("session"),
           "reports": grade_reports(reps, truth, cfg["calls"]),
           "tasks": grade_tasks(trec, cfg["calls"], cfg.get("zones", []), truth, bands),
           "condition": condition_summary(jl(f["condition"])) if f["condition"] else None,
           "tolerances": TOL}
    rs = out["reports"]
    ts = out["tasks"]
    out["summary"] = {
        "calls_reported": sum(r["reported"] for r in rs), "calls": len(rs),
        "missed_p1_p2": [r["call"] for r in rs if not r["reported"] and r["truth_priority"] in ("P1", "P2")],
        "fields_correct": sum(r.get("fields_correct", 0) for r in rs),
        "fields_scored_reported": sum(r.get("fields_scored", 0) for r in rs),
        "priority_correct": sum(1 for r in rs if r.get("fields", {}).get("priority")),
        "tasks_shown": sum(1 for t in ts if t["status"] in ("answered", "expired")),
        "tasks_answered": sum(1 for t in ts if t["status"] == "answered"),
        "tasks_correct": sum(1 for t in ts if t.get("correct")),
        "task_median_response_s": _median([t["response_s"] for t in ts if t.get("response_s") is not None]),
    }
    path = os.path.join(d, f"grades_{rid}.json")
    json.dump(out, open(path, "w"), indent=1)
    s = out["summary"]
    print(f"{rid} set {out['set_id']}: reported {s['calls_reported']}/{s['calls']} calls, "
          f"fields {s['fields_correct']}/{s['fields_scored_reported']}, missed P1/P2 {s['missed_p1_p2']}; "
          f"tasks answered {s['tasks_answered']}/{s['tasks_shown']}, correct {s['tasks_correct']} -> {path}")
    return out


def _median(x):
    x = sorted(x)
    return None if not x else (x[len(x) // 2] if len(x) % 2 else (x[len(x) // 2 - 1] + x[len(x) // 2]) / 2)


def main():
    a = sys.argv[1:]
    if not a:
        sys.exit(__doc__)
    if a[0] == "--all":
        rows = []
        for d in sorted(glob.glob(os.path.join(DATA, "runs", "*"))):
            g = grade(d)
            if g:
                rows.append({"run_id": g["run_id"], "set_id": g["set_id"],
                             **{k: g["session"].get(k) for k in ("participant", "block", "condition", "tau_s")},
                             **{k: v for k, v in g["summary"].items() if not isinstance(v, list)},
                             "missed_p1_p2": " ".join(g["summary"]["missed_p1_p2"]),
                             "condition_final": (g["condition"] or {}).get("final_pct")})
        if rows:
            p = os.path.join(DATA, "grades_all.csv")
            with open(p, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)
            print(f"-> {p}")
    else:
        grade(a[0])


if __name__ == "__main__":
    main()
