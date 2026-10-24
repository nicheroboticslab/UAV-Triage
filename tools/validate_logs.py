#!/usr/bin/env python3
"""
Check one run's logs for completeness and consistency (H6/H7 data quality).

    python3 tools/validate_logs.py                 # newest run in ~/uav_dataset
    python3 tools/validate_logs.py 20261023_133632 # a specific run
    python3 tools/validate_logs.py --dir PATH ID

A run is the set of files sharing one stamp:
  flight_<id>.jsonl    LLM decisions (one per agent decision)
  inaction_<id>.jsonl  session header + inaction episodes (A7)
  battery_<id>.jsonl   config header + 2 Hz samples (D6)
  controls_<id>.jsonl  10 Hz stick/button/gimbal samples
  nofly_<id>.jsonl     no-fly zone events (legacy; replaced by condition_ when a call set is used)
  condition_<id>.jsonl Condition config + 1 Hz samples + impact / no-fly events (G1)
  reports_<id>.jsonl   report portal: config, field changes, submissions (E4)
  tasks_<id>.jsonl     secondary tasks: shown / opened / answer / expired (H5)
  meta_<id>.json       provenance snapshot (H6/H7)

Prints PASS / WARN / FAIL lines and a summary; exit code 1 if any FAIL.
"""
import glob
import json
import math
import os
import sys

DATA = os.path.expanduser("~/uav_dataset")
results = []


def out(level, msg):
    results.append(level)
    print(f"[{level}] {msg}")


def load(path):
    rows, bad = [], 0
    with open(path) as f:
        for i, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                bad += 1
    return rows, bad


def gaps(ts, nominal, factor=3.0):
    """Sample intervals longer than factor x nominal: [(t_before, gap_s)]."""
    return [(a, b - a) for a, b in zip(ts, ts[1:]) if b - a > factor * nominal]


def check_inaction(rows, live_s):
    head = [r for r in rows if r.get("type") == "session"]
    eps = [r for r in rows if r.get("type") == "episode"]
    if len(head) != 1:
        out("FAIL", f"inaction: expected 1 session header, found {len(head)}")
        return None, eps
    h = head[0]
    out("PASS", f"inaction: header tau={h['tau_s']} control={h['is_control']} call_set={h['block'].get('call_set')} "
                f"session_sha256={h['session_sha256'][:12]}…")
    for k in ("participant", "sequence", "block"):
        if h["block"].get(k) is None:
            out("WARN", f"inaction: block.{k} is null (fill session.json for study runs)")
    tau = h["tau_s"]
    prev_end, n_ok = -1.0, True
    for e in eps:
        if e["start_s"] < prev_end - 1e-3:
            out("FAIL", f"inaction: episode {e['n']} starts before episode {e['n'] - 1} ends"); n_ok = False
        if e["end_s"] < e["start_s"]:
            out("FAIL", f"inaction: episode {e['n']} ends before it starts"); n_ok = False
        if abs((e["end_s"] - e["start_s"]) - e["duration_s"]) > 0.01:
            out("FAIL", f"inaction: episode {e['n']} duration mismatch"); n_ok = False
        if tau is not None:
            should = e["duration_s"] > tau + 0.2
            if e["tau_reached"] and e["takeover_s"] is None:
                out("FAIL", f"inaction: episode {e['n']} tau_reached but no takeover_s"); n_ok = False
            if e["tau_reached"]:
                lag = e["takeover_s"] - e["start_s"] - tau
                if not (-0.05 <= lag <= 0.3):
                    out("WARN", f"inaction: episode {e['n']} takeover {lag:+.2f} s from tau")
            elif should and e["ended_by"] != "shutdown":
                out("FAIL", f"inaction: episode {e['n']} lasted {e['duration_s']:.1f} s > tau but tau_reached=false"); n_ok = False
        if live_s is not None and e["start_s"] < live_s - 0.5:
            out("FAIL", f"inaction: episode {e['n']} starts at {e['start_s']:.1f} s, before control live ({live_s:.1f} s)"); n_ok = False
        prev_end = e["end_s"]
    if n_ok:
        out("PASS", f"inaction: {len(eps)} episodes, ordered, durations consistent, tau logic consistent")
    if eps and eps[-1]["ended_by"] != "shutdown":
        out("WARN", "inaction: last episode not closed by shutdown (run may not have ended cleanly)")
    take = sum(1 for e in eps if e.get("tau_reached"))
    out("INFO", f"inaction: {take} takeover(s); ended_by = {sorted({e['ended_by'] for e in eps})}")
    return h, eps


def check_battery(rows):
    cfg = [r for r in rows if r.get("type") == "config"]
    s = [r for r in rows if "t_s" in r]
    if len(cfg) != 1:
        out("FAIL", f"battery: expected 1 config header, found {len(cfg)}")
    if not s:
        out("FAIL", "battery: no samples"); return s
    ts = [r["t_s"] for r in s]
    g = gaps(ts, 0.5)
    out("PASS" if not g else "WARN", f"battery: {len(s)} samples, {ts[0]:.1f}–{ts[-1]:.1f} s"
        + ("" if not g else f"; {len(g)} gaps, largest {max(x[1] for x in g):.1f} s at t={max(g, key=lambda x: x[1])[0]:.1f}"))
    up = [(a["t_s"], a["pct"], b["pct"]) for a, b in zip(s, s[1:]) if b["pct"] > a["pct"] + 1e-6]
    out("PASS" if not up else "FAIL", "battery: percentage never increases" if not up else f"battery: pct increases {len(up)} times")
    diff = [abs(r["pct"] - r["px4_pct"]) for r in s if r.get("px4_pct") is not None]
    if diff:
        out("PASS" if max(diff) < 3 else "WARN", f"battery: QGC/PX4 value tracks model (max diff {max(diff):.1f} %)")
    nulls = sorted({k for r in s for k, v in r.items() if v is None and not (k == "px4_pct" and r["t_s"] < 3.0)})
    if nulls:
        out("WARN", f"battery: null fields present: {nulls}")
    return s


def check_controls(rows):
    if not rows:
        out("WARN", "controls: no file or empty (no controller connected?)"); return rows
    ts = [r["t_s"] for r in rows]
    g = gaps(ts, 0.1)
    out("PASS" if not g else "WARN", f"controls: {len(rows)} samples, {ts[0]:.1f}–{ts[-1]:.1f} s, "
        f"mean rate {len(ts) / max(1e-6, ts[-1] - ts[0]):.1f} Hz"
        + ("" if not g else f"; {len(g)} gaps, largest {max(x[1] for x in g):.2f} s"))
    gp = [r["gimbal_pitch_deg"] for r in rows]
    gy = [r["gimbal_yaw_deg"] for r in rows]
    ok = all(0 <= p <= 90 for p in gp) and all(-90 <= y <= 90 for y in gy)
    out("PASS" if ok else "FAIL", f"controls: gimbal within limits (pitch {min(gp):.0f}–{max(gp):.0f}, yaw {min(gy):.0f}–{max(gy):.0f})")
    sp = [math.hypot(*r["cmd_vel_ned"][:2]) for r in rows if r.get("cmd_vel_ned")]
    if sp:
        out("PASS" if max(sp) <= 23.01 else "FAIL", f"controls: commanded horizontal speed max {max(sp):.1f} m/s (cap 23)")
    return rows


def check_flight(rows):
    if not rows:
        out("INFO", "flight: no LLM decisions (agent never engaged?)"); return rows
    need = ("model", "model_digest", "options", "prompt_sha256", "prompt_code_sha256")
    miss = [r["id"] for r in rows if not all(k in (r.get("llm_output", {}).get("llm_meta") or {}) for k in need)
            and not r.get("llm_output", {}).get("fallback_used")]
    out("PASS" if not miss else "FAIL", f"flight: {len(rows)} decisions; llm_meta complete" if not miss
        else f"flight: llm_meta missing in {miss}")
    dig = {r["llm_output"]["llm_meta"].get("model_digest") for r in rows if r.get("llm_output", {}).get("llm_meta")}
    opts = {json.dumps(r["llm_output"]["llm_meta"].get("options"), sort_keys=True) for r in rows if r.get("llm_output", {}).get("llm_meta")}
    out("PASS" if len(dig) <= 1 and len(opts) <= 1 else "FAIL", f"flight: one model digest and one option set across decisions ({len(dig)}, {len(opts)})")
    fb = sum(1 for r in rows if r.get("llm_output", {}).get("fallback_used"))
    na = sum(1 for r in rows if r.get("llm_output", {}).get("applied") is False)
    rt = [r["llm_output"].get("response_time_seconds") for r in rows if r.get("llm_output", {}).get("response_time_seconds")]
    out("INFO", f"flight: fallbacks {fb}, discarded (applied=false) {na}, LLM latency "
        + (f"{min(rt):.1f}–{max(rt):.1f} s" if rt else "n/a"))
    ts = [r["timestamp"] for r in rows]
    out("PASS" if ts == sorted(ts) else "FAIL", "flight: decisions in time order")
    strnull = sorted({k for r in rows for k, v in r.get("input", {}).items() if v == "None"})
    if strnull:
        out("WARN", f"flight: fields with the string \"None\" instead of null: {strnull}")
    th = {(r["input"].get("battery_low_thresh"), r["input"].get("battery_crit_thresh")) for r in rows}
    out("INFO", f"flight: battery thresholds given to the LLM {sorted(th)} (battery gate: see battery.json)")
    return rows


def check_condition(rows):
    if not rows:
        return
    cfg = [r for r in rows if r.get("type") == "config"]
    out("PASS" if cfg else "FAIL", f"condition: config header {'present' if cfg else 'missing'}"
        + (f" ({len(cfg[0].get('zones', []))} no-fly zones)" if cfg else ""))
    smp = [r for r in rows if r.get("type") == "sample"]
    g = gaps([r["t_s"] for r in smp], 1.0)
    out("PASS" if smp and not g else "WARN", f"condition: {len(smp)} samples, {len(g)} gap(s) > 3 s")
    ev = {}
    for r in rows:
        if r.get("type") not in ("config", "sample"):
            ev[r["type"]] = ev.get(r["type"], 0) + 1
    last = smp[-1]["condition_pct"] if smp else None
    out("INFO", f"condition: final {last} %, events {ev}")
    if any(r.get("condition_pct", 100) < 0 or r.get("condition_pct", 0) > 100 for r in rows if "condition_pct" in r):
        out("FAIL", "condition: value outside 0-100")


def check_portal(reps, tasks):
    if not reps and not tasks:
        return
    cfg = next((r for r in reps if r.get("type") == "config"), None)
    out("PASS" if cfg else "FAIL", f"reports: config header {'present' if cfg else 'missing'}")
    subs = [r for r in reps if r.get("type") == "submit"]
    fields = (cfg or {}).get("fields", {})
    badv = [r["call"] for r in subs if any(k in fields and v not in fields[k] for k, v in r["fields"].items())]
    out("PASS" if not badv else "FAIL", f"reports: {len(subs)} submission(s) for {len({r['call'] for r in subs})} call(s)"
        + (f"; invalid values for {badv}" if badv else ""))
    nosnap = sum(1 for r in subs if not (r.get("snapshot") or {}).get("pos_ned"))
    if nosnap:
        out("WARN", f"reports: {nosnap} submission(s) without vehicle position (pilot state missing?)")
    lst = next((r["tasks"] for r in tasks if r.get("type") == "task_list"), [])
    final = {}
    for r in tasks:
        if r.get("task") and r.get("type") in ("shown", "answer", "expired", "not_shown"):
            final[r["task"]] = r["type"]
    cnt = {}
    for t in lst:
        cnt[final.get(t["id"], "pending")] = cnt.get(final.get(t["id"], "pending"), 0) + 1
    out("INFO", f"tasks: {len(lst)} scripted, outcome {cnt}")
    if cnt.get("shown"):
        out("WARN", f"tasks: {cnt['shown']} task(s) still open at the end of the log (portal stopped before expiry)")


def check_meta(d, rid):
    p = os.path.join(d, f"meta_{rid}.json")
    if not os.path.exists(p):
        out("WARN", "meta: file missing (H6 provenance)")
        return
    m = json.load(open(p))
    g = m.get("git", {})
    dirty = g.get("status_porcelain") or []
    out("PASS" if not dirty else "WARN", f"meta: git {str(g.get('commit'))[:10]} on {g.get('branch')}"
        + (f", {len(dirty)} uncommitted file(s)" if dirty else ", clean"))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    d = sys.argv[sys.argv.index("--dir") + 1] if "--dir" in sys.argv else DATA
    if "--dir" in sys.argv:
        args = [a for a in args if a != d]
    if args:
        rid = args[0]
    else:
        files = sorted(glob.glob(os.path.join(d, "inaction_*.jsonl")), key=os.path.getmtime)
        if not files:
            sys.exit(f"No runs in {d}")
        rid = os.path.basename(files[-1])[len("inaction_"):-len(".jsonl")]
    print(f"Run {rid} in {d}\n")

    data = {}
    for kind in ("flight", "inaction", "battery", "controls", "nofly", "condition", "reports", "tasks"):
        p = os.path.join(d, f"{kind}_{rid}.jsonl")
        if os.path.exists(p):
            rows, bad = load(p)
            data[kind] = rows
            if bad:
                out("FAIL", f"{kind}: {bad} unparseable line(s)")
        else:
            data[kind] = []
            if kind in ("nofly", "condition", "reports", "tasks"):
                out("INFO", f"{kind}: no file")
            else:
                out("WARN" if kind in ("flight", "controls") else "FAIL", f"{kind}: file missing")

    # control-live time from the decision event logs, if present
    live = None
    for r in data["flight"]:
        for ev in r.get("input", {}).get("event_log_at_decision", []):
            if "Control live" in ev:
                t = ev.split("]")[0].strip("[T+")
                m, s_ = (t.split("m") + ["0s"])[:2] if "m" in t else ("0", t)
                live = int(m or 0) * 60 + int(s_.rstrip("s") or 0)
                break
        if live is not None:
            break

    h, eps = check_inaction(data["inaction"], live)
    b = check_battery(data["battery"])
    c = check_controls(data["controls"])
    check_flight(data["flight"])
    check_condition(data["condition"])
    check_portal(data["reports"], data["tasks"])
    check_meta(d, rid)
    if data["nofly"]:
        out("WARN", f"nofly: {len(data['nofly'])} zone events — no-fly zones are dropped from the design but still loaded")

    # cross-file
    ends = {k: v[-1]["t_s"] for k, v in (("battery", b), ("controls", c)) if v}
    if eps:
        ends["inaction"] = eps[-1]["end_s"]
    if len(ends) > 1:
        spread = max(ends.values()) - min(ends.values())
        lvl = "PASS" if spread < 2 else ("INFO" if spread < 60 else "WARN")   # block end -> landing can take a while
        out(lvl, f"cross: logs end within {spread:.2f} s of each other {ends}")
    if b and eps:
        auto = {round(r["t_s"]) for r in b if r.get("egle_state") == "AUTONOMOUS"}
        for e in eps:
            if e.get("tau_reached"):
                t0 = round(e["takeover_s"]) + 1
                if t0 <= b[-1]["t_s"] and not any(t in auto for t in range(t0, t0 + 3)):
                    out("WARN", f"cross: takeover at {e['takeover_s']:.1f} s not followed by AUTONOMOUS in battery log")
    ntake = sum(1 for e in eps if e.get("tau_reached"))
    nloss = max([r["input"].get("signal_loss_count", 0) for r in data["flight"]] or [0])
    if data["flight"]:
        out("PASS" if nloss <= ntake else "WARN", f"cross: takeovers in inaction log {ntake}, highest count in decisions {nloss}")

    print()
    nf, nw = results.count("FAIL"), results.count("WARN")
    print(f"Summary: {results.count('PASS')} pass, {nw} warn, {nf} fail")
    sys.exit(1 if nf else 0)


if __name__ == "__main__":
    main()
