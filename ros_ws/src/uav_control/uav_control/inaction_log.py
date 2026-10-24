"""
Operator inaction-episode log (requirement A7).

An episode starts at the operator's last input and is opened once the
operator has been idle for episode_min_s. It closes at the next operator
input (or at shutdown, marked as censored). Every episode is written to
~/uav_dataset/inaction_<timestamp>.jsonl, one JSON object per line:

  {"type": "session", ...}   first line: tau, session file path and hash
  {"type": "episode", ...}   one per inaction episode

state_before_input is the vehicle state just before the episode ended
(the agent's state if it had taken over); for shutdown it is the state
at shutdown.

open_task (A7, §1.5) comes from the report portal (report_portal.py), which
publishes /tmp/uav_portal_state.json: what the participant was editing when the
episode started ("report:C2", "task:T05" or null), the requests pending then,
whether the portal window had focus, and how many portal actions happened during
the episode. Portal use itself is never operator input.
"""
import json
import os
import time
from datetime import datetime

PORTAL_STATE_FILE = "/tmp/uav_portal_state.json"


class InactionLog:
    def __init__(self, dataset_dir, session, flight_start_time):
        os.makedirs(dataset_dir, exist_ok=True)
        sid = datetime.fromtimestamp(flight_start_time).strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(dataset_dir, f"inaction_{sid}.jsonl")
        self.t0 = flight_start_time
        self.tau = session["tau_s"]
        self.min_s = session["episode_min_s"]
        self.count = 0
        self.open = None
        self._portal = {}
        self._portal_read = 0.0
        self._write({
            "type": "session",
            "wall_time": time.time(),
            "flight_id": f"flight_{sid}",
            "tau_s": self.tau,
            "is_control": session["is_control"],
            "episode_min_s": self.min_s,
            "session_file": session["path"],
            "session_sha256": session["sha256"],
            "block": session["block"],
        })

    def _write(self, record):
        with open(self.filepath, "a") as f:
            f.write(json.dumps(record) + "\n")

    def _read_portal(self, now):
        if now - self._portal_read >= 0.5:
            self._portal_read = now
            try:
                with open(PORTAL_STATE_FILE) as f:
                    p = json.load(f)
                self._portal = p if now - p.get("wall", 0) < 5 else {}
            except Exception:
                self._portal = {}
        return self._portal

    def update(self, now, last_input_time, state):
        """Call every control loop with the current time and the last operator input time."""
        portal = self._read_portal(now)
        if self.open is None:
            if now - last_input_time >= self.min_s:
                self.open = {"start": last_input_time, "state_at_start": state,
                             "last_state": state, "takeover": None,
                             "open_task": portal.get("editing"),
                             "pending_tasks": portal.get("pending_tasks", []),
                             "portal_focus": portal.get("focus"),
                             "portal_event_at_open": portal.get("last_event_wall"),
                             "portal_events": set(), "tasks_seen": set()}
        elif last_input_time > self.open["start"]:
            # Input arrived. The input thread may already have switched the
            # state to MANUAL, so report the last state seen while idle.
            self._close(last_input_time, self.open["last_state"], "operator_input")
        else:
            self.open["last_state"] = state
            if portal.get("last_event_wall") and portal.get("last_event_wall") != self.open["portal_event_at_open"]:
                self.open["portal_events"].add(portal["last_event_wall"])
            if portal.get("editing"):
                self.open["tasks_seen"].add(portal["editing"])

    def mark_takeover(self, now):
        """Call when the agent engages because tau was reached."""
        if self.open is not None and self.open["takeover"] is None:
            self.open["takeover"] = now

    def close(self, now, state, reason="shutdown"):
        if self.open is not None:
            self._close(now, state, reason)

    def _close(self, end, state, reason):
        ep = self.open
        self.open = None
        self.count += 1
        to = ep["takeover"]
        self._write({
            "type": "episode",
            "n": self.count,
            "start_s": round(ep["start"] - self.t0, 3) + 0.0,
            "end_s": round(end - self.t0, 3) + 0.0,
            "duration_s": round(end - ep["start"], 3),
            "start_wall": ep["start"],
            "end_wall": end,
            "tau_s": self.tau,
            "tau_reached": None if self.tau is None else to is not None,
            "takeover_s": None if to is None else round(to - self.t0, 3),
            "state_at_start": ep["state_at_start"],
            "state_before_input": state,
            "open_task": ep["open_task"],
            "tasks_during": sorted(ep["tasks_seen"]),
            "pending_tasks_at_start": ep["pending_tasks"],
            "portal_focus_at_start": ep["portal_focus"],
            "portal_actions_during": len(ep["portal_events"]),
            "ended_by": reason,
        })
