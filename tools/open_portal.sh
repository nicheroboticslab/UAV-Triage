#!/bin/bash
# Open the participant dashboard / report portal (E4) full-screen on the table monitor.
# Uses its own browser profile ("uav_portal_profile"), so stop_session.sh can close
# exactly this window without touching any other browser window.
#
#   tools/open_portal.sh                 # http://127.0.0.1:8765/
#   UAV_PORTAL_POS=1920,0 tools/open_portal.sh   # top-left corner of the table monitor (Chromium only)
#   UAV_PORTAL_BROWSER=firefox tools/open_portal.sh
#
# Chromium/Chrome is preferred (--app window, position can be set). Firefox works too:
# it opens in kiosk mode on the monitor where it starts.

URL="${UAV_PORTAL_URL:-http://127.0.0.1:8765/}"
POS="${UAV_PORTAL_POS:-}"
# default position: the portal monitor from displays.json (tools/displays.py)
if [ -z "$POS" ]; then
    read -r PX PY PW PH < <(python3 "$(dirname "$0")/displays.py" geom portal) && POS="$PX,$PY"
fi

# wait for the portal server
for _ in $(seq 1 40); do
    curl -s -o /dev/null "$URL" && break
    sleep 0.25
done

pick() {
    if [ -n "$UAV_PORTAL_BROWSER" ]; then echo "$UAV_PORTAL_BROWSER"; return; fi
    for b in chromium chromium-browser google-chrome google-chrome-stable firefox; do
        command -v "$b" > /dev/null && { echo "$b"; return; }
    done
}
B=$(pick)
case "$B" in
    *chrom*)
        PROFILE="$HOME/.cache/uav_portal_profile"
        mkdir -p "$PROFILE"
        ARGS=(--user-data-dir="$PROFILE" --app="$URL" --no-first-run --disable-session-crashed-bubble
              --disable-infobars --noerrdialogs --start-fullscreen)
        [ -n "$POS" ] && ARGS+=(--window-position="$POS")
        exec "$B" "${ARGS[@]}"
        ;;
    firefox)
        # snap Firefox can only use profiles inside ~/snap/firefox/common
        if [ -d "$HOME/snap/firefox" ]; then PROFILE="$HOME/snap/firefox/common/uav_portal_profile"
        else PROFILE="$HOME/.cache/uav_portal_profile_ff"; fi
        mkdir -p "$PROFILE"
        exec firefox -no-remote -profile "$PROFILE" --kiosk "$URL"
        ;;
    *)
        echo "No browser found (chromium / google-chrome / firefox). Open $URL by hand."
        ;;
esac
