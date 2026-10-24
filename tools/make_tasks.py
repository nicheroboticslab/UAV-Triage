#!/usr/bin/env python3
"""
Generate the scripted secondary tasks for each call set (E4 / H5, Ben 24 Oct).

    python3 tools/make_tasks.py           # BASELINE and A-E -> calls/tasks/<SET>.json
    python3 tools/make_tasks.py --check   # print only

Secondary tasks are incident-command / dispatch requests that appear in the
report portal during the block, on top of reporting. They are scripted, not
random: every set gets the same request types at the same times (seconds after
START), and requests that refer to a call refer to the call at the same route
position, so the sets stay matched (decisions log §6.2, §7.3). There are more
requests than an operator can answer while also flying and reporting; each one
expires after EXPIRE_S if it is not answered.

Every answer is a fixed entry (a choice or a number) so it can be graded
automatically afterwards (tools/grade_reports.py), from the state logged with
the answer:
  ACK          acknowledge a dispatch notice                  -> answered before expiry
  STATUS       report battery % and Condition %               -> within +-3 of the values shown at submission
  NEAREST      which call is closest to you now               -> nearest call pin at submission
  HOME_BEARING which direction is home base from you          -> 8-point compass bearing at submission
  NOFLY_LINE   is a no-fly zone on the straight line to call X -> geometry at submission
  DISTANCE     how far is call X from home base               -> distance band (static)
  BOAT         which reported call should the one boat team go to first
                                                              -> highest ground-truth priority (P1/P2) among
                                                                 calls reported so far; also graded for
                                                                 consistency with the operator's own reports
None of these give feedback to the participant (no live score, §8.3).
"""
import json
import os
import sys

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
SETS = ["BASELINE", "A", "B", "C", "D", "E"]
EXPIRE_S = 60
# (time after START in s, kind, route position 1-5 for call-specific requests)
SCHEDULE = [
    (45, "ACK", None), (95, "STATUS", None), (140, "NEAREST", None), (190, "NOFLY_LINE", 3),
    (235, "HOME_BEARING", None), (285, "DISTANCE", 5), (330, "BOAT", None), (380, "ACK", 3),
    (425, "NEAREST", None), (475, "STATUS", None), (520, "NOFLY_LINE", 4), (570, "HOME_BEARING", None),
    (615, "DISTANCE", 2), (665, "BOAT", None), (710, "ACK", 5), (760, "STATUS", None),
]
ACK_TEXT = [
    "Dispatch: Engine 3 is staging at home base. Acknowledge.",
    "Dispatch: a swift-water rescue team has been requested for the area near {call}. Acknowledge.",
    "Dispatch: police have closed the roads around {call}. Acknowledge.",
]
DIST_BANDS = [[0, 300, "Under 300 m"], [300, 600, "300-600 m"], [600, 900, "600-900 m"], [900, 99999, "Over 900 m"]]
COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]


ACK_POOL = [
    "Dispatch: a second ambulance is staging at home base. Acknowledge.",
    "Dispatch: utility crews are cutting power near {call}. Acknowledge.",
    "Dispatch: Red Cross shelter opened at the fieldhouse. Acknowledge.",
    "Dispatch: a boat team is launching upstream of {call}. Acknowledge.",
    "Dispatch: road closures expanded around {call}. Acknowledge.",
    "Dispatch: National Weather Service extended the flash flood warning. Acknowledge.",
    "Dispatch: Engine 7 is responding to {call}. Acknowledge.",
    "Dispatch: a news helicopter is in the area, stay clear. Acknowledge.",
    "Dispatch: water rescue requested near {call}. Acknowledge.",
    "Dispatch: incident command moved to the parking structure. Acknowledge.",
    "Dispatch: police checking vehicles near {call}. Acknowledge.",
    "Dispatch: rain intensity increasing over the next hour. Acknowledge.",
    "Dispatch: medical team en route to home base. Acknowledge.",
    "Dispatch: dam operators report rising levels near {call}. Acknowledge.",
    "Dispatch: shift change at dispatch in 10 minutes. Acknowledge.",
]


def schedule_for(set_id):
    """BASELINE keeps the original schedule. A-E (Ben, 24 Oct): same counts of every
    request type, but their own order, times (+-15 s jitter, >= 35 s apart), route
    positions and notice texts, so the sequence cannot be learned across blocks.
    Deterministic (seeded by set id)."""
    if set_id == "BASELINE":
        return SCHEDULE, ACK_TEXT
    import random
    rnd = random.Random("egle-tasks-" + set_id)
    kinds = [k for _, k, _ in SCHEDULE]
    for _ in range(10000):                    # no request type twice in a row
        rnd.shuffle(kinds)
        if all(kinds[i] != kinds[i + 1] for i in range(len(kinds) - 1)):
            break
    times, t = [], None
    for base, _, _ in SCHEDULE:
        x = base + rnd.randint(-15, 15)
        if t is not None and x < t + 35:
            x = t + 35
        times.append(x)
        t = x
    pos = {"NOFLY_LINE": rnd.sample(range(1, 6), 2), "DISTANCE": rnd.sample(range(1, 6), 2),
           "ACK": [None] + rnd.sample(range(1, 6), 2)}
    used = {k: 0 for k in pos}
    sched = []
    for tm, k in zip(times, kinds):
        rp = None
        if k in pos:
            rp = pos[k][used[k]]
            used[k] += 1
        sched.append((tm, k, rp))
    idx = "ABCDE".index(set_id)
    texts = [ACK_POOL[(3 * idx + j) % len(ACK_POOL)] for j in range(3)]
    texts = sorted(texts, key=lambda x: "{call}" in x)        # first notice has no call reference
    return sched, texts


def build(set_id):
    cs = json.load(open(os.path.join(REPO, "calls", "sets", f"{set_id}.json")))
    by_loc = {c["location"]: c["call_id"] for c in cs["calls"]}
    order = cs.get("route_order") or [c["location"] for c in cs["calls"]]
    route_calls = [by_loc[L] for L in order]
    call_ids = sorted(by_loc.values())
    tasks, n_ack = [], 0
    sched, ack_text = schedule_for(set_id)
    for i, (t, kind, rp) in enumerate(sched):
        call = route_calls[rp - 1] if rp else None
        task = {"id": f"T{i + 1:02d}", "t_s": t, "expire_s": EXPIRE_S, "kind": kind, "call": call,
                "route_pos": rp}
        if kind == "ACK":
            task["text"] = ack_text[n_ack % len(ack_text)].format(call=call)
            task["fields"] = [{"name": "ack", "type": "choice", "options": ["Acknowledged"]}]
            n_ack += 1
        elif kind == "STATUS":
            task["text"] = "Incident command: report your vehicle status."
            task["fields"] = [{"name": "battery_pct", "label": "Battery %", "type": "number", "min": 0, "max": 100},
                              {"name": "condition_pct", "label": "Condition %", "type": "number", "min": 0, "max": 100}]
        elif kind == "NEAREST":
            task["text"] = "Incident command: which call location are you closest to right now?"
            task["fields"] = [{"name": "call", "type": "choice", "options": call_ids}]
        elif kind == "HOME_BEARING":
            task["text"] = "Ground crew: in which direction is home base from your position?"
            task["fields"] = [{"name": "bearing", "type": "choice", "options": COMPASS}]
        elif kind == "NOFLY_LINE":
            task["text"] = f"Air operations: is a no-fly zone on the straight line between you and {call}?"
            task["fields"] = [{"name": "nofly", "type": "choice", "options": ["Yes", "No"]}]
        elif kind == "DISTANCE":
            task["text"] = f"Swift-water team at home base: how far is {call} from home base?"
            task["fields"] = [{"name": "band", "type": "choice", "options": [b[2] for b in DIST_BANDS]}]
        elif kind == "BOAT":
            task["text"] = ("Incident command has one boat team. Which of the calls you have reported "
                            "should it go to first?")
            task["fields"] = [{"name": "call", "type": "choice", "options": call_ids + ["No call needs a boat"]}]
        tasks.append(task)
    return {"_help": "Scripted secondary tasks (generated by tools/make_tasks.py; matched across sets). "
                     "t_s = seconds after START; each request expires after expire_s.",
            "set_id": set_id, "dist_bands": DIST_BANDS, "compass": COMPASS, "tasks": tasks}


def main():
    out = os.path.join(REPO, "calls", "tasks")
    os.makedirs(out, exist_ok=True)
    for s in SETS:
        d = build(s)
        print(f"{s:9s} {len(d['tasks'])} tasks: " + " ".join(f"{t['kind']}{'(' + t['call'] + ')' if t['call'] else ''}"
                                                         for t in d["tasks"]))
        if "--check" not in sys.argv:
            json.dump(d, open(os.path.join(out, f"{s}.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
