from pymavlink import mavutil
import time

# Connect to PX4 SITL
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected — system {master.target_system} component {master.target_component}")

# PX4 SITL default home GPS origin
HOME_LAT = 47.397742
HOME_LON = 8.545594

def ned_to_gps(north, east, alt_m):
    lat = HOME_LAT + (north / 111320.0)
    lon = HOME_LON + (east  / (111320.0 * 0.7174))
    return lat, lon, alt_m

# Sectors in NED frame — matches your autonomy script exactly
sectors = [
    ("BASE",     0,    0,    5),
    ("SECTOR_A", 100,  0,    10),
    ("SECTOR_B", 50,   100,  10),
    ("SECTOR_C", 0,   -100,  10),
    ("SECTOR_D", -100, 0,    10),
    ("SECTOR_E", 80,   80,   10),
]

# Send count
master.mav.mission_count_send(
    master.target_system,
    master.target_component,
    len(sectors),
    0
)

time.sleep(0.5)

for i, (name, n, e, alt) in enumerate(sectors):
    lat, lon = ned_to_gps(n, e, alt)[:2]
    print(f"  WP{i} {name}: lat={lat:.6f} lon={lon:.6f} alt={alt}m")
    master.mav.mission_item_int_send(
        master.target_system,
        master.target_component,
        i,                          # sequence
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,                          # current
        1,                          # autocontinue
        3, 0, 0, 0,                 # hold, acceptance radius, pass radius, yaw
        int(lat * 1e7),
        int(lon * 1e7),
        float(alt)
    )
    time.sleep(0.2)

print("\nAll 6 waypoints uploaded — check QGC Plan view map")
