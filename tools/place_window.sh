#!/bin/bash
# Move a window onto its assigned monitor and make it full screen (X11).
#   bash tools/place_window.sh "EGLe Camera" camera
#   bash tools/place_window.sh "EGLe Operator Station" portal
# Monitor per role: displays.json (see tools/displays.py). Uses wmctrl if installed,
# else xdotool. Waits up to 60 s for the window to appear. Does nothing if the
# monitor is not connected.
TITLE="$1"; ROLE="$2"
REPO_DIR="$HOME/UAV-Triage"
read -r X Y W H < <(python3 "$REPO_DIR/tools/displays.py" geom "$ROLE") || { echo "[place] $ROLE: monitor not connected"; exit 0; }
[ -z "$X" ] && { echo "[place] $ROLE: monitor not connected"; exit 0; }

for _ in $(seq 1 120); do
    if command -v wmctrl > /dev/null; then
        wmctrl -l | grep -qF "$TITLE" && break
    elif command -v xdotool > /dev/null; then
        xdotool search --name "$TITLE" > /dev/null 2>&1 && break
    else
        echo "[place] neither wmctrl nor xdotool installed (sudo apt install wmctrl)"; exit 0
    fi
    sleep 0.5
done

if command -v wmctrl > /dev/null; then
    wmctrl -r "$TITLE" -b remove,fullscreen,maximized_vert,maximized_horz
    wmctrl -r "$TITLE" -e "0,$X,$Y,$W,$H"
    sleep 0.3
    wmctrl -r "$TITLE" -b add,fullscreen
else
    for wid in $(xdotool search --name "$TITLE"); do
        xdotool windowmove "$wid" "$X" "$Y" windowsize "$wid" "$W" "$H"
    done
fi
echo "[place] '$TITLE' -> $ROLE monitor ${W}x${H}+${X}+${Y}"
