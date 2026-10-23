#!/bin/bash
# Start Gazebo with the generated world for a call set (E3 test path).
# PX4 started afterwards joins this running world and binds to egle_drone.
#   bash tools/start_world.sh BASELINE
set -e
SET=${1:-BASELINE}
REPO="$HOME/UAV-Triage"
PX4="$HOME/PX4-Autopilot"
python3 "$REPO/tools/make_world.py" "$SET"
export GZ_SIM_RESOURCE_PATH="$REPO/models:$PX4/Tools/simulation/gz/models:$PX4/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH}"
echo "Starting Gazebo with worlds/egle_${SET}.sdf (first run downloads Fuel models; can take a few minutes)"
gz sim -r "$REPO/worlds/egle_${SET}.sdf"
