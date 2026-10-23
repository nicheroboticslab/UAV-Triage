#!/bin/bash
# UAV-Triage Config A — launcher with real readiness checks
#
# Every stage opens as its own separate gnome-terminal window running a
# generated script file. Chaining multiple --tab blocks in one gnome-terminal
# call proved unreliable on this system (a later --tab's command could get
# mis-assigned or duplicated), so this version avoids that entirely: one
# process, one window, no chaining, no inline multi-line command strings.
#
# Usage:
#   chmod +x run_config_a.sh
#   ./run_config_a.sh

REPO_DIR="$HOME/UAV-Triage"
LOG_DIR="$REPO_DIR/logs"
TAB_DIR="$LOG_DIR/tab_scripts"
PX4_LOG="$LOG_DIR/px4_session.log"

mkdir -p "$TAB_DIR"
> "$PX4_LOG"   # truncate any stale log so old "Ready for takeoff!" lines can't cause a false match

echo "Checking for leftover processes from a previous run..."
if pgrep -f "px4|MicroXRCEAgent|QGroundControl|autonomous_uav|gz sim" > /dev/null; then
    echo "WARNING: found existing related processes still running:"
    ps aux | grep -E "px4|MicroXRCEAgent|QGroundControl|autonomous_uav|gz sim" | grep -v grep
    read -p "Kill them and continue? (y/N) " confirm
    if [[ "$confirm" == "y" || "$confirm" == "Y" ]]; then
        pkill -9 -f "px4|MicroXRCEAgent|QGroundControl|autonomous_uav|gz sim"
        sleep 2
    else
        echo "Aborted. Resolve manually, then rerun."
        exit 1
    fi
fi

# Polls a log file for a pattern, with a visible countdown and a hard timeout.
wait_for_log() {
    local pattern="$1"
    local logfile="$2"
    local timeout="$3"
    local waited=0
    echo -n "Waiting for: \"$pattern\" "
    while ! grep -qF "$pattern" "$logfile" 2>/dev/null; do
        sleep 1
        waited=$((waited + 1))
        echo -n "."
        if [ "$waited" -ge "$timeout" ]; then
            echo ""
            echo "TIMEOUT after ${timeout}s waiting for: \"$pattern\""
            echo "Check the relevant window directly — something may be stuck."
            return 1
        fi
    done
    echo " confirmed (${waited}s)"
    return 0
}

# --- Write each stage's command to its own script file ---

cat > "$TAB_DIR/1_dds_bridge.sh" << EOF
#!/bin/bash
MicroXRCEAgent udp4 -p 8888
exec bash
EOF

cat > "$TAB_DIR/2_px4_gazebo.sh" << EOF
#!/bin/bash
sleep 2
eval "\$(python3 ~/UAV-Triage/scenario_tool.py --env)"   # home location from scenario.json
cd ~/PX4-Autopilot
stdbuf -oL -eL make px4_sitl gz_x500 2>&1 | tee "$PX4_LOG"
exec bash
EOF

cat > "$TAB_DIR/3_qgc.sh" << EOF
#!/bin/bash
"$REPO_DIR/QGroundControl.AppImage"
exec bash
EOF

cat > "$TAB_DIR/4_pilot.sh" << EOF
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/UAV-Triage/ros_ws/install/local_setup.bash
cd ~/UAV-Triage/ros_ws
python3 src/uav_control/uav_control/autonomous_uav_stage5.py
exec bash
EOF

cat > "$TAB_DIR/5_dashboard.sh" << EOF
#!/bin/bash
echo -en "\033]0;Dashboard\a"
source /opt/ros/humble/setup.bash
source ~/UAV-Triage/ros_ws/install/local_setup.bash
python3 ~/UAV-Triage/ros_ws/src/uav_control/uav_control/uav_dashboard.py
exec bash
EOF

chmod +x "$TAB_DIR"/*.sh

# --- Launch, one window per stage, gated by real readiness checks ---

echo ""
echo "=== Stage 1/3: DDS bridge + PX4/Gazebo (2 windows) ==="
gnome-terminal --window --title="1-DDS-Bridge" -- bash "$TAB_DIR/1_dds_bridge.sh"
gnome-terminal --window --title="2-PX4-Gazebo" -- bash "$TAB_DIR/2_px4_gazebo.sh"

wait_for_log "Startup script returned successfully" "$PX4_LOG" 120 || exit 1

echo ""
echo "Setting PX4 parameters..."
python3 "$REPO_DIR/set_px4_params.py" || echo "  WARNING: parameter check failed — run 'param show GF_ACTION' in the PX4 window before START"
echo ""
# Upload BEFORE QGroundControl starts, so QGC loads markers and zones when it connects
echo "Uploading scenario to the map (from scenario.json)..."
python3 "$REPO_DIR/upload_waypoints.py" > "$LOG_DIR/upload_waypoints.log" 2>&1 \
    && echo "  sector markers: OK" || echo "  sector markers: FAILED (see logs/upload_waypoints.log)"
python3 "$REPO_DIR/upload_nofly.py" > "$LOG_DIR/upload_nofly.log" 2>&1 \
    && echo "  no-fly zones:   OK" || echo "  no-fly zones:   FAILED (see logs/upload_nofly.log)"
echo ""

echo ""
echo "=== Stage 2/3: QGroundControl ==="
gnome-terminal --window --title="3-QGroundControl" -- bash "$TAB_DIR/3_qgc.sh"

wait_for_log "Ready for takeoff!" "$PX4_LOG" 60
if [ $? -ne 0 ]; then
    echo "QGroundControl likely hasn't connected yet. Check its window — once it"
    echo "shows a connected vehicle, launch the pilot script and dashboard manually:"
    echo "  bash $TAB_DIR/4_pilot.sh"
    echo "  bash $TAB_DIR/5_dashboard.sh"
    exit 1
fi

echo ""
echo "=== Stage 3/3: Dashboard, then pilot script on START ==="
rm -f /tmp/uav_dashboard_state.json   # don't show the previous run's state while waiting
gnome-terminal --window --title="Dashboard" -- bash "$TAB_DIR/5_dashboard.sh"

echo ""
echo "Simulation ready. Drone is on the ground, disarmed."
echo ""
python3 "$REPO_DIR/start_gate.py" || { echo "Start cancelled — simulation left running."; exit 1; }

gnome-terminal --window --title="4-Pilot-Script" -- bash "$TAB_DIR/4_pilot.sh"

echo ""
echo "Mission started. Click into the Dashboard window before pressing any flight keys."
