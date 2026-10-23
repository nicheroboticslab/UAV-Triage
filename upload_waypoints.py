"""
Upload scenario sectors to PX4 so they appear in QGroundControl's Plan view.
Reads ~/UAV-Triage/scenario.json — edit that file, not this one.

    python3 upload_waypoints.py            # upload
    python3 upload_waypoints.py --dry-run  # preview only
"""
import json, os, sys, time
from pymavlink import mavutil

PATH = os.environ.get("UAV_SCENARIO", os.path.expanduser("~/UAV-Triage/scenario.json"))
sc = json.load(open(PATH))
h = sc["home"]; cruise = float(sc.get("cruise_alt_m", 10))

items = [("BASE (" + h.get("name", "home") + ")", h["lat"], h["lon"], float(sc.get("base", {}).get("alt_m", 5)))]
for s in sorted(sc["sectors"], key=lambda s: s["priority"]):
    items.append((f'{s["name"]} {s.get("label","")}'.strip(), s["lat"], s["lon"], float(s.get("alt_m", cruise))))

print(f"Scenario: {PATH}")
print("QGC#  NAME")
for i, (name, lat, lon, alt) in enumerate(items):
    print(f"{i+1:>4}  {name:<34} {lat:.6f}, {lon:.6f}  alt {alt:.0f} m")
if "--dry-run" in sys.argv:
    sys.exit(0)

master = mavutil.mavlink_connection("udp:127.0.0.1:14540")
print("\nWaiting for heartbeat on 14540...")
if master.wait_heartbeat(timeout=10) is None:
    sys.exit("No heartbeat — is the simulation running?")
ts, tc = master.target_system, master.target_component

master.mav.mission_count_send(ts, tc, len(items), 0)
sent = set(); deadline = time.time() + 5 * (len(items) + 1)
while len(sent) < len(items):
    if time.time() > deadline:
        sys.exit("TIMEOUT: PX4 did not request all items.")
    msg = master.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST"], blocking=True, timeout=5)
    if msg is None:
        sys.exit("TIMEOUT waiting for PX4 to request the next item.")
    if msg.seq in sent or msg.seq >= len(items):
        continue
    _, lat, lon, alt = items[msg.seq]
    master.mav.mission_item_int_send(ts, tc, msg.seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1, 3, 0, 0, 0, int(lat * 1e7), int(lon * 1e7), alt)
    sent.add(msg.seq)

ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
if ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
    sys.exit(f"Upload not acknowledged (ack={getattr(ack, 'type', None)}).")
print(f"Uploaded {len(items)} markers — see QGC Plan view.")
