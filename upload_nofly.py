"""
Upload no-fly zones (PX4 geofence exclusion areas) to PX4 so they appear on the
researcher's QGroundControl map (Plan > Fence tab).

Source: when session.json names a call set, calls/nofly/<SET>.json (the zones the
Condition monitor, the agent and the projector minimap use); otherwise the legacy
"no_fly_zones" in scenario.json. --set X overrides.

Usage (simulation running):
    python3 upload_nofly.py            # upload the zones of the session's call set
    python3 upload_nofly.py --set B    # a given set
    python3 upload_nofly.py --dry-run  # show what would be uploaded, no connection
    python3 upload_nofly.py --clear    # remove all zones from the vehicle

IMPORTANT: set PX4's breach action to "warning only" once (see instructions),
otherwise a breach can switch PX4 out of Offboard mode and fight the pilot script.
"""
import json
import os
import sys
import time

os.environ["MAVLINK20"] = "1"   # fence uploads need MAVLink 2 (mission_type field)
from pymavlink import mavutil

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
PATH = os.environ.get("UAV_SCENARIO", os.path.join(REPO, "scenario.json"))
SET = sys.argv[sys.argv.index("--set") + 1] if "--set" in sys.argv else None
if SET is None:
    try:
        SET = json.load(open(os.environ.get("UAV_SESSION", os.path.join(REPO, "session.json"))))["block"].get("call_set")
    except Exception:
        SET = None
if SET:
    nf = os.path.join(REPO, "calls", "nofly", f"{SET}.json")
    try:
        ZONES = {"circles": [{"name": f'{z["id"]} {z["name"]}', "radius_m": z["radius_m"], "lat": z["lat"], "lon": z["lon"]}
                             for z in json.load(open(nf))["zones"]]}
        print(f"Call set {SET}: zones from {nf}")
    except FileNotFoundError:
        print(f"Call set {SET}: no {nf}; uploading no zones")
        ZONES = {}
else:
    ZONES = json.load(open(PATH)).get("no_fly_zones", {})
    print(f"No call set: zones from {PATH}")
FENCE = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
FRAME = mavutil.mavlink.MAV_FRAME_GLOBAL

# Build fence items: (command, param1, lat, lon, description)
items = []
for c in ZONES.get("circles", []):
    items.append((mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                  float(c["radius_m"]), c["lat"], c["lon"], f'circle "{c["name"]}" r={c["radius_m"]}m'))
for p in ZONES.get("polygons", []):
    pts = p["points"]
    if len(pts) < 3:
        sys.exit(f'Polygon "{p["name"]}" needs at least 3 points')
    for i, (lat, lon) in enumerate(pts):
        items.append((mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                      float(len(pts)), lat, lon, f'polygon "{p["name"]}" vertex {i+1}/{len(pts)}'))

print(f"{len(items)} fence items:")
if not items and "--clear" not in sys.argv:
    sys.argv.append("--clear")          # nothing to show: make sure old zones are removed
for i, (_, _, lat, lon, desc) in enumerate(items):
    print(f"  {i:2d}  {lat:.6f}, {lon:.6f}  {desc}")
if "--dry-run" in sys.argv:
    sys.exit(0)

master = mavutil.mavlink_connection("udp:127.0.0.1:14540")
print("\nWaiting for heartbeat on 14540...")
if master.wait_heartbeat(timeout=10) is None:
    sys.exit("No heartbeat — is the simulation running?")
ts, tc = master.target_system, master.target_component

def wait_ack(label):
    ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    if ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        sys.exit(f"{label}: not acknowledged (ack={getattr(ack, 'type', None)})")

if "--clear" in sys.argv:
    master.mav.mission_clear_all_send(ts, tc, FENCE)
    wait_ack("Clear")
    print("All no-fly zones removed.")
    sys.exit(0)

master.mav.mission_count_send(ts, tc, len(items), FENCE)
sent = set()
deadline = time.time() + 5 * (len(items) + 1)
while len(sent) < len(items):
    if time.time() > deadline:
        sys.exit("TIMEOUT: PX4 did not request all fence items.")
    msg = master.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST"], blocking=True, timeout=5)
    if msg is None:
        sys.exit("TIMEOUT waiting for PX4 to request the next fence item.")
    if getattr(msg, "mission_type", FENCE) != FENCE or msg.seq in sent or msg.seq >= len(items):
        continue
    cmd, p1, lat, lon, _ = items[msg.seq]
    master.mav.mission_item_int_send(ts, tc, msg.seq, FRAME, cmd, 0, 1,
                                     p1, 0, 0, 0, int(lat * 1e7), int(lon * 1e7), 0.0, FENCE)
    sent.add(msg.seq)

wait_ack("Upload")
print(f"Uploaded {len(items)} fence items — check QGC's map (Plan > Fence tab).")
