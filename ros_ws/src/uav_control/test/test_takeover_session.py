"""Offline tests for session_config and inaction_log (no ROS / PX4 needed).

    cd ~/UAV-Triage/ros_ws/src/uav_control
    python3 -m pytest test/test_takeover_session.py -q
"""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "uav_control"))
from session_config import load_session  # noqa: E402
from inaction_log import InactionLog  # noqa: E402


def _session_file(tmp_path, tau=15, levels=(5, 15, 25, 35), episode_min=1.0):
    p = tmp_path / "session.json"
    p.write_text(json.dumps({"tau_levels_s": list(levels), "episode_min_s": episode_min,
                             "block": {"participant": "P01", "block": 2, "tau_s": tau}}))
    return str(p)


def _records(path):
    with open(path) as f:
        return [json.loads(line) for line in f]


# ---- session_config ----

def test_tau_loaded(tmp_path):
    s = load_session(_session_file(tmp_path, tau=25))
    assert s["tau_s"] == 25.0 and not s["is_control"] and s["warnings"] == []


def test_control_condition(tmp_path):
    s = load_session(_session_file(tmp_path, tau=None))
    assert s["tau_s"] is None and s["is_control"]


def test_off_level_tau_warns(tmp_path):
    assert load_session(_session_file(tmp_path, tau=12))["warnings"]


@pytest.mark.parametrize("bad", [0, -5, "15", True])
def test_invalid_tau_rejected(tmp_path, bad):
    with pytest.raises(ValueError):
        load_session(_session_file(tmp_path, tau=bad))


def test_missing_file_rejected(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_session(str(tmp_path / "nope.json"))


def test_missing_tau_key_rejected(tmp_path):
    p = tmp_path / "session.json"
    p.write_text(json.dumps({"block": {}}))
    with pytest.raises(ValueError):
        load_session(str(p))


# ---- inaction_log ----

def _log(tmp_path, tau=15):
    s = load_session(_session_file(tmp_path, tau=tau))
    return InactionLog(str(tmp_path), s, flight_start_time=1000.0)


def test_session_header(tmp_path):
    log = _log(tmp_path, tau=15)
    head = _records(log.filepath)[0]
    assert head["type"] == "session" and head["tau_s"] == 15.0 and head["block"]["participant"] == "P01"


def test_short_gap_not_logged(tmp_path):
    log = _log(tmp_path)
    last = 1000.0
    for i in range(1, 40):          # input every 0.5 s, below episode_min_s
        now = 1000.0 + i * 0.5
        last = now
        log.update(now + 0.2, last, "MANUAL")
    assert len(_records(log.filepath)) == 1


def test_episode_without_takeover(tmp_path):
    log = _log(tmp_path, tau=15)
    log.update(1003.0, 1000.0, "MANUAL")      # idle 3 s -> opens at last input (t=0)
    log.update(1008.0, 1008.0, "MANUAL")      # input at t=8 -> closes
    ep = _records(log.filepath)[1]
    assert str(ep["start_s"]) == "0.0" and ep["end_s"] == 8.0 and ep["duration_s"] == 8.0
    assert ep["tau_reached"] is False and ep["takeover_s"] is None and ep["ended_by"] == "operator_input"


def test_episode_with_takeover(tmp_path):
    log = _log(tmp_path, tau=5)
    log.update(1002.0, 1000.0, "MANUAL")
    log.mark_takeover(1005.1)
    log.update(1010.0, 1000.0, "AUTO_FLYING")   # agent flying, still no input
    log.update(1020.0, 1020.0, "MANUAL")        # input thread already switched to MANUAL
    ep = _records(log.filepath)[1]
    assert ep["tau_reached"] is True and ep["takeover_s"] == 5.1 and ep["duration_s"] == 20.0
    assert ep["state_before_input"] == "AUTO_FLYING"


def test_control_tau_reached_is_none(tmp_path):
    log = _log(tmp_path, tau=None)
    log.update(1050.0, 1000.0, "MANUAL")
    log.update(1060.0, 1060.0, "MANUAL")
    ep = _records(log.filepath)[1]
    assert ep["tau_s"] is None and ep["tau_reached"] is None and ep["duration_s"] == 60.0


def test_shutdown_censors_open_episode(tmp_path):
    log = _log(tmp_path)
    log.update(1002.0, 1000.0, "MANUAL")
    log.close(1030.0, "AUTO_FLYING")
    ep = _records(log.filepath)[1]
    assert ep["ended_by"] == "shutdown" and ep["duration_s"] == 30.0
    assert ep["state_before_input"] == "AUTO_FLYING"


def test_consecutive_episodes_numbered(tmp_path):
    log = _log(tmp_path)
    log.update(1002.0, 1000.0, "MANUAL")
    log.update(1004.0, 1004.0, "MANUAL")
    log.update(1007.0, 1004.0, "MANUAL")
    log.update(1009.0, 1009.0, "MANUAL")
    eps = _records(log.filepath)[1:]
    assert [e["n"] for e in eps] == [1, 2] and eps[1]["start_s"] == 4.0
