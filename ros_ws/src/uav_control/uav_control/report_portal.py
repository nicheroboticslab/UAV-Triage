#!/usr/bin/env python3
"""
Participant dashboard + report portal for the table monitor (E4, E9, H5, H8).

    python3 report_portal.py              # serves http://127.0.0.1:8765/
    python3 report_portal.py --port 8765

A small local web app (Python standard library only; the page is plain
HTML/JS) opened full-screen on the table monitor. It is the only place the
participant uses keyboard and mouse.

Shows (E9, decisions log §9.6): EGLe state + THINKING tag, the agent's intent
label and target (no reasoning text), battery, Condition, flight time; the
call list (call ID, call text, distance, reported or not); the report form for
the active call; incoming incident-command requests (secondary tasks).

Report form (E4, §7.6-7.8): fixed entries only, so every report can be scored
automatically: Priority P1-P4 · People 0-5+ · Water depth None/Low/Medium/Deep
(asked only when people > 0) · Water motion (Standing/Flowing; None/Standing/
Flowing when nobody is present) · Worst condition (only when people > 0).
The active call is chosen automatically: the nearest call pin within
REPORT_RADIUS_M; it is locked while the form is being filled in. A call can be
reported again; every submission is logged and the grader uses the last one.

Secondary tasks: calls/tasks/<SET>.json (tools/make_tasks.py), shown at their
scripted times after START and expired if not answered.

Portal use is NOT operator input: nothing here touches the takeover timer.
What the participant is doing in the portal is published to
/tmp/uav_portal_state.json, which the pilot script's inaction log reads, so
each inaction episode records the open task (A7, §1.5).

Logs (~/uav_dataset, named by the run id of the pilot's flight so the archive
picks them up): reports_<run>.jsonl, tasks_<run>.jsonl. Each record carries a
snapshot of the vehicle state (position, battery, Condition, EGLe state) at
that moment; tools/grade_reports.py scores them.
"""
import json
import math
import os
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import call_sets  # noqa: E402
from log_time import stamped  # noqa: E402

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
DATASET = os.path.expanduser(os.environ.get("UAV_DATASET", "~/uav_dataset"))
STATE_FILE = "/tmp/uav_dashboard_state.json"
PORTAL_STATE_FILE = "/tmp/uav_portal_state.json"
PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "report_portal.html")
REPORT_RADIUS_M = 150.0
PORT = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 8765

INTENT_LABEL = {"HOLD_SAFE": "Hold position", "CONTINUE_ASSIGNED_TASK": "Continue current call",
                "SEARCH_NEXT_PRIORITY": "Go to next call", "INSPECT_TARGET_REPORT": "Inspect call",
                "COMMS_REACQUIRE": "Reacquire link", "RETURN_TO_BASE": "Return home",
                "ABORT_AND_SAFE": "Land now"}
STATE_TEXT = {"MANUAL": "YOU ARE FLYING", "AUTONOMOUS": "AGENT IS FLYING", "RETURNING": "RETURNING HOME",
              "HOLD": "HOLDING", "LANDED": "LANDED"}

# Field definitions shared with the page and the grader
FIELDS = {
    "priority": ["P1", "P2", "P3", "P4"],
    "people": ["0", "1", "2", "3", "4", "5+"],
    "water_depth": ["None", "Low", "Medium", "Deep"],
    "water_motion": ["None", "Standing", "Flowing"],
    "worst": ["none", "mobile", "immobile_responsive", "unresponsive"],
}


def load_config():
    sess_path = os.environ.get("UAV_SESSION", os.path.join(REPO, "session.json"))
    sess = json.load(open(sess_path))
    set_id = sess["block"].get("call_set")
    sc = json.load(open(os.path.join(REPO, "scenario.json")))
    cfg = {"session": sess["block"], "set_id": set_id, "calls": [], "tasks": [], "zones": []}
    if not set_id:
        return cfg
    cs, calls = call_sets.resolve(set_id, sc["home"], float(sc.get("cruise_alt_m", 10.0)))
    text = {c["call_id"]: c.get("details", "") for c in cs["calls"]}
    cfg["calls"] = [{"id": c["call_id"], "text": text.get(c["call_id"], ""), "location": c["location"],
                     "n": c["pos"][0], "e": c["pos"][1]} for c in sorted(calls, key=lambda c: c["call_id"])]
    try:
        cfg["tasks"] = json.load(open(os.path.join(REPO, "calls", "tasks", f"{set_id}.json")))["tasks"]
    except FileNotFoundError:
        print(f"[portal] no calls/tasks/{set_id}.json: no secondary tasks")
    try:
        cfg["zones"] = json.load(open(os.path.join(REPO, "calls", "nofly", f"{set_id}.json")))["zones"]
    except FileNotFoundError:
        pass
    return cfg


class Portal:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.RLock()
        self.state = {}
        self.run_id = None
        self.t0 = None
        self.reports = {}            # call id -> last submitted fields
        self.revisions = {}          # call id -> number of submissions
        self.task_status = {}        # task id -> shown / answered / expired / not_shown
        self.locked_call = None
        self.editing = None
        self.focus = None
        self.last_event_wall = None
        self.f_rep = self.f_task = None

    # ---------------- logging
    def _open_logs(self):
        os.makedirs(DATASET, exist_ok=True)
        self.f_rep = open(os.path.join(DATASET, f"reports_{self.run_id}.jsonl"), "a")
        self.f_task = open(os.path.join(DATASET, f"tasks_{self.run_id}.jsonl"), "a")
        head = {"type": "config", "run_id": self.run_id, "flight_start_wall": self.t0,
                "session": self.cfg["session"], "set_id": self.cfg["set_id"], "calls": self.cfg["calls"],
                "zones": self.cfg["zones"], "report_radius_m": REPORT_RADIUS_M, "fields": FIELDS}
        for f in (self.f_rep, self.f_task):
            self._w(f, dict(head))
        self._w(self.f_task, {"type": "task_list", "tasks": self.cfg["tasks"]})

    def _w(self, f, rec):
        if f is None:
            return
        rec = stamped(rec)
        if self.t0:
            rec.setdefault("t_s", round(rec["wall"] - self.t0, 3))
        f.write(json.dumps(rec) + "\n")
        f.flush()

    def snapshot(self):
        s = self.state
        return {"pos_ned": s.get("position"), "battery_pct": s.get("battery_pct"),
                "condition_pct": s.get("condition_pct"), "egle_state": s.get("egle_state"),
                "drone_state": s.get("drone_state"), "current_wp": s.get("current_wp"),
                "nofly_zone": s.get("nofly_zone"), "state_age_s": round(time.time() - s.get("timestamp", 0), 2)}

    # ---------------- background: state file, run start, task schedule
    def poll(self):
        while True:
            try:
                with open(STATE_FILE) as f:
                    st = json.load(f)
            except Exception:
                st = None
            with self.lock:
                if st:
                    self.state = st
                    fsw = st.get("flight_start_wall")
                    if fsw and self.t0 != fsw and time.time() - st.get("timestamp", 0) < 5:
                        self.t0 = fsw
                        self.run_id = st.get("run_id") or datetime.fromtimestamp(fsw).strftime("%Y%m%d_%H%M%S")
                        self._open_logs()
                        print(f"[portal] run {self.run_id}: logging reports and tasks")
                self._schedule()
                self._publish()
            time.sleep(0.25)

    def ended(self):
        return bool(self.state.get("block_ended")) or self.state.get("egle_state") == "LANDED"

    def _schedule(self):
        if not self.t0 or self.f_task is None:
            return
        t = time.time() - self.t0
        for task in self.cfg["tasks"]:
            st = self.task_status.get(task["id"])
            if st is None and self.ended():
                self.task_status[task["id"]] = "not_shown"
                self._w(self.f_task, {"type": "not_shown", "task": task["id"], "reason": "block_ended"})
            elif st is None and t >= task["t_s"] + task["expire_s"]:     # portal was not running then
                self.task_status[task["id"]] = "not_shown"
                self._w(self.f_task, {"type": "not_shown", "task": task["id"], "reason": "portal_not_running"})
            elif st is None and t >= task["t_s"]:
                self.task_status[task["id"]] = "shown"
                self._w(self.f_task, {"type": "shown", "task": task["id"], "kind": task["kind"],
                                      "snapshot": self.snapshot()})
            elif st == "shown" and (t >= task["t_s"] + task["expire_s"] or self.ended()):
                self.task_status[task["id"]] = "expired"
                self._w(self.f_task, {"type": "expired", "task": task["id"],
                                      "reason": "block_ended" if self.ended() else "timeout",
                                      "snapshot": self.snapshot()})

    def pending(self):
        t = time.time() - self.t0 if self.t0 else 0
        out = []
        for task in self.cfg["tasks"]:
            if self.task_status.get(task["id"]) == "shown":
                out.append({**task, "remaining_s": max(0, round(task["t_s"] + task["expire_s"] - t))})
        return out

    def _publish(self):
        rec = {"wall": time.time(), "run_id": self.run_id, "editing": self.editing, "focus": self.focus,
               "pending_tasks": [t["id"] for t in self.pending()], "last_event_wall": self.last_event_wall,
               "reported_calls": sorted(self.reports)}
        try:
            tmp = PORTAL_STATE_FILE + ".tmp"
            with open(tmp, "w") as f:
                json.dump(rec, f)
            os.replace(tmp, PORTAL_STATE_FILE)
        except Exception:
            pass

    # ---------------- view model for the page
    def view(self):
        with self.lock:
            s = self.state
            pos = s.get("position") or [0.0, 0.0, 0.0]
            calls = []
            nearest = None
            for c in self.cfg["calls"]:
                d = math.hypot(c["n"] - pos[0], c["e"] - pos[1])
                calls.append({"id": c["id"], "text": c["text"], "dist_m": round(d),
                              "reported": c["id"] in self.reports, "revisions": self.revisions.get(c["id"], 0)})
                if d <= REPORT_RADIUS_M and (nearest is None or d < nearest[1]):
                    nearest = (c["id"], d)
            active = self.locked_call or (nearest[0] if nearest else None)
            eg = s.get("egle_state")
            fresh = time.time() - s.get("timestamp", 0) < 5
            phase = "waiting" if not self.t0 or not fresh else ("ended" if self.ended() else "flying")
            intent = INTENT_LABEL.get(s.get("current_intent") or "", "") if eg == "AUTONOMOUS" else ""
            wp = s.get("current_wp")
            return {
                "phase": phase,
                "status": {"state": eg, "state_text": STATE_TEXT.get(eg, eg or "—"),
                           "thinking": bool(s.get("llm_running")) and eg == "AUTONOMOUS",
                           "intent": intent, "target": wp if (intent and wp not in (None, "None", "BASE")) else None,
                           "battery_pct": s.get("battery_pct"), "battery_low": s.get("battery_low"),
                           "condition_pct": s.get("condition_pct"), "nofly_zone": s.get("nofly_zone"),
                           "flight_time": s.get("flight_time")},
                "block": self.cfg["session"].get("block"),
                "calls": calls, "active_call": active, "locked": bool(self.locked_call),
                "report_radius_m": REPORT_RADIUS_M, "fields": FIELDS,
                "tasks": self.pending(),
            }

    # ---------------- events from the page
    def event(self, kind, body):
        with self.lock:
            self.last_event_wall = time.time()
            if kind == "report_event":                     # open / field / cancel
                ev = body.get("event")
                call = body.get("call")
                if ev == "field" and self.locked_call is None:
                    self.locked_call = call
                if ev == "cancel":
                    self.locked_call = None
                self.editing = f"report:{call}" if self.locked_call else None
                self._w(self.f_rep, {"type": ev, "call": call, "field": body.get("field"),
                                     "value": body.get("value"), "snapshot": self.snapshot()})
                self._publish()
                return {"ok": True}
            if kind == "report_submit":
                call, fields = body.get("call"), body.get("fields", {})
                err = self._validate(fields)
                if err:
                    return {"ok": False, "error": err}
                self.reports[call] = fields
                self.revisions[call] = self.revisions.get(call, 0) + 1
                self._w(self.f_rep, {"type": "submit", "call": call, "fields": fields,
                                     "revision": self.revisions[call], "snapshot": self.snapshot()})
                self.locked_call = None
                self.editing = None
                self._publish()
                return {"ok": True}
            if kind == "task_answer":
                tid, values = body.get("task"), body.get("values", {})
                if self.task_status.get(tid) != "shown":
                    return {"ok": False, "error": "This request has expired."}
                self.task_status[tid] = "answered"
                self._w(self.f_task, {"type": "answer", "task": tid, "values": values,
                                      "reports_so_far": dict(self.reports), "snapshot": self.snapshot()})
                if self.editing == f"task:{tid}":
                    self.editing = None
                self._publish()
                return {"ok": True}
            if kind == "task_event":                       # first interaction with a request
                tid = body.get("task")
                self.editing = f"task:{tid}" if not self.locked_call else self.editing
                self._w(self.f_task, {"type": "opened", "task": tid, "snapshot": self.snapshot()})
                self._publish()
                return {"ok": True}
            if kind == "focus":                            # page focus / blur
                self.focus = bool(body.get("focus"))
                self._w(self.f_rep, {"type": "focus", "focus": self.focus})
                self._publish()
                return {"ok": True}
            return {"ok": False, "error": "unknown"}

    @staticmethod
    def _validate(f):
        for k in ("priority", "people", "water_motion"):
            if f.get(k) not in FIELDS[k]:
                return f"Missing: {k}"
        if f["people"] != "0":
            if f.get("water_depth") not in FIELDS["water_depth"] or f.get("worst") not in FIELDS["worst"][1:]:
                return "Missing: water depth or worst condition"
            if (f["water_depth"] == "None") != (f["water_motion"] == "None"):
                return "Water motion must be None exactly when depth is None"
        return None


def make_handler(portal):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(200, open(PAGE, "rb").read(), "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._send(200, portal.view())
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
            except Exception:
                return self._send(400, {"ok": False, "error": "bad request"})
            kind = self.path.rsplit("/", 1)[-1]
            self._send(200, portal.event(kind, body))
    return H


def main():
    cfg = load_config()
    portal = Portal(cfg)
    threading.Thread(target=portal.poll, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), make_handler(portal))
    print(f"[portal] set {cfg['set_id']}: {len(cfg['calls'])} calls, {len(cfg['tasks'])} secondary tasks; "
          f"http://127.0.0.1:{PORT}/")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
