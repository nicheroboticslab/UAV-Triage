#!/bin/bash
# Stop a run_config_a.sh session: end the pilot script cleanly (logs closed),
# kill the simulator processes, and close every terminal window the launcher
# opened. The terminal run_config_a.sh was started from is left open.
# Called by the dashboard on Q Q; can also be run by hand.

REPO_DIR="$HOME/UAV-Triage"
TAB_DIR="$REPO_DIR/logs/tab_scripts"
PATTERN="px4|MicroXRCEAgent|QGroundControl|autonomous_uav|gz sim|camera_view|uav_dashboard|flow_animator|report_portal|uav_portal_profile"

echo "[stop] $(date '+%F %T') stopping session"

# 1. Pilot script: SIGINT so it closes the inaction/battery logs (its finally block).
if pgrep -f autonomous_uav_stage5 > /dev/null; then
    pkill -INT -f autonomous_uav_stage5
    for _ in $(seq 1 10); do
        pgrep -f autonomous_uav_stage5 > /dev/null || break
        sleep 0.5
    done
fi

# 2. Everything else in the stack (same set the launcher clears at start).
pkill -9 -f "$PATTERN"
sleep 1

# 2b. H6: archive this run (every logger is stopped now)
python3 "$REPO_DIR/tools/archive_run.py" --latest || echo "[stop] archive failed (run tools/archive_run.py --pending later)"
# 2c. Grade the archived run's reports and secondary tasks (J1 components)
LATEST=$(ls -1 "$HOME/uav_dataset/runs" 2>/dev/null | sort | tail -1)
[ -n "$LATEST" ] && python3 "$REPO_DIR/tools/grade_reports.py" "$HOME/uav_dataset/runs/$LATEST"

# 3. Close the launcher's windows. Each window's shell recorded its PID and
#    carries UAV_TAB_SESSION; only PIDs that still carry this session's id are
#    closed, so a stale PID that now belongs to something else is never touched.
SID=$(cat "$TAB_DIR/session_id" 2>/dev/null)
if [ -n "$SID" ] && [ -f "$TAB_DIR/session_pids" ]; then
    while read -r pid; do
        [ -r "/proc/$pid/environ" ] || continue
        if tr '\0' '\n' < "/proc/$pid/environ" | grep -qx "UAV_TAB_SESSION=$SID"; then
            kill -HUP "$pid" 2>/dev/null
        fi
    done < "$TAB_DIR/session_pids"
fi
rm -f /tmp/uav_dashboard_state.json /tmp/uav_start_signal /tmp/uav_portal_state.json
echo "[stop] done"
