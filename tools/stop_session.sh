#!/bin/bash
# Stop a run_config_a.sh session: end the pilot script cleanly (logs closed),
# kill the simulator processes, and close every terminal window the launcher
# opened. The terminal run_config_a.sh was started from is left open.
# Called by the dashboard on Q Q; can also be run by hand.

REPO_DIR="$HOME/UAV-Triage"
TAB_DIR="$REPO_DIR/logs/tab_scripts"
PATTERN="px4|MicroXRCEAgent|QGroundControl|autonomous_uav|gz sim|camera_view|uav_dashboard"

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
rm -f /tmp/uav_dashboard_state.json
echo "[stop] done"
