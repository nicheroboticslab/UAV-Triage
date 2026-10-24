#!/usr/bin/env python3
"""
Set up session.json for one participant block (H4).

    python3 tools/session_setup.py P07 3        # participant P07, block 3
    python3 tools/session_setup.py P07          # show P07's six blocks
    python3 tools/session_setup.py --test SET [TAU|control]   # dev/test run, no participant

Looks the participant up in calls/sequences.json (Graeco-Latin square and
participant assignment from Experiment Design) and writes session.json:
participant, sequence, block, condition, call_set and tau_s. Keeps
tau_levels_s / episode_min_s. Prints what it wrote; the run's inaction log
records the session file's hash.
"""
import json
import os
import sys

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
SEQ = os.path.join(REPO, "calls", "sequences.json")
SESSION = os.environ.get("UAV_SESSION", os.path.join(REPO, "session.json"))
SETS = os.path.join(REPO, "calls", "sets")


def show(pid, seq, rows):
    print(f"{pid}: sequence {seq}")
    for r in rows:
        tau = "control (no takeover)" if r["tau_s"] is None else f"tau {r['tau_s']} s"
        print(f"  block {r['block']}: set {r['call_set']:8s} {tau}   [{r['condition']}]")


def write(block):
    cfg = json.load(open(SESSION)) if os.path.exists(SESSION) else {
        "tau_levels_s": [2, 5, 12, 30], "episode_min_s": 1.0}
    if not os.path.exists(os.path.join(SETS, f"{block['call_set']}.json")):
        sys.exit(f"Call set {block['call_set']} not found in calls/sets")
    cfg["block"] = block
    with open(SESSION, "w") as f:
        json.dump(cfg, f, indent=2)
        f.write("\n")
    print(f"Wrote {SESSION}:")
    print("  " + json.dumps(block))


def main():
    a = sys.argv[1:]
    if not a:
        sys.exit(__doc__)
    if a[0] == "--test":
        if len(a) < 2:
            sys.exit("usage: session_setup.py --test SET [TAU|control]")
        tau = None if len(a) < 3 or a[2] == "control" else float(a[2])
        write({"participant": None, "sequence": None, "block": None, "condition": "test",
               "call_set": a[1], "tau_s": tau})
        return
    d = json.load(open(SEQ))
    pid = a[0].upper()
    if pid not in d["participant_assignment"]:
        sys.exit(f"{pid} not in participant_assignment (P01-P{len(d['participant_assignment']):02d})")
    seq = d["participant_assignment"][pid]
    rows = d["sequences"][str(seq)]
    if len(a) == 1:
        show(pid, seq, rows)
        return
    b = int(a[1])
    r = next((r for r in rows if r["block"] == b), None)
    if r is None:
        sys.exit(f"Block {b} not in sequence {seq} (1-{len(rows)})")
    write({"participant": pid, "sequence": seq, "block": b, "condition": r["condition"],
           "call_set": r["call_set"], "tau_s": r["tau_s"]})
    show(pid, seq, [r])


if __name__ == "__main__":
    main()
