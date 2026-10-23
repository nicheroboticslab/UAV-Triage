"""
Set required PX4 parameters on every launch (PX4 SITL isn't keeping them).
Reads each value back from PX4 to confirm.

    python3 set_px4_params.py
"""
import struct
import sys
import time

from pymavlink import mavutil

# name: (value, type)
PARAMS = {
    "GF_ACTION": (1, "int"),   # geofence breach -> warning only (never take control from the pilot script)
    # D1: DJI Matrice 30T-class limits (decisions log 3.1). The documented PX4
    # ranges are XY <= 20 and Z down <= 4; the read-back below shows whether
    # PX4 holds the M30T values.
    "MPC_XY_VEL_MAX": (23.0, "real"),    # m/s horizontal cap (operator and agent)
    "MPC_Z_VEL_MAX_UP": (6.0, "real"),   # m/s ascent
    "MPC_Z_VEL_MAX_DN": (5.0, "real"),   # m/s descent
    "RTL_RETURN_ALT": (30.0, "real"),    # m, emergency return (LB+RB) altitude
    # D4: PX4's own autonomous cruise speed (used by return-to-launch) matches
    # the agent's fixed cruise speed (AGENT_CRUISE_MS in the pilot script).
    "MPC_XY_CRUISE": (14.0, "real"),
    # D2: PX4's simulated battery (shown in QGC) starts full; the pilot script
    # then lowers SIM_BAT_MIN_PCT to the battery model's % (px4_battery_display.py).
    "SIM_BAT_MIN_PCT": (100.0, "real"),
    # PX4's own low-battery failsafe stays warning-only (0), so it cannot override
    # the scripted return gate (D3). Thresholds left at PX4 defaults (15/7/5 %).
    "COM_LOW_BAT_ACT": (0, "int"),
}

INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32


def encode_int(v, how):
    # "bytewise": int bits packed into the float field; "cast": plain float(v)
    return struct.unpack("<f", struct.pack("<i", v))[0] if how == "bytewise" else float(v)


def decode_int(f):
    """Return (bytewise_value, cast_value) for a received float field."""
    return struct.unpack("<i", struct.pack("<f", f))[0], int(round(f))


def read_back(master, name, timeout=3):
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is not None:
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
            if pid.rstrip("\x00") == name:
                return msg
    return None


def main():
    master = mavutil.mavlink_connection("udp:127.0.0.1:14540")
    if master.wait_heartbeat(timeout=10) is None:
        sys.exit("No heartbeat from PX4.")
    ts, tc = master.target_system, master.target_component
    failed = False
    for name, (value, kind) in PARAMS.items():
        ok = False
        tries = ("bytewise", "cast") if kind == "int" else ("real",)
        for how in tries:
            if kind == "int":
                master.mav.param_set_send(ts, tc, name.encode(), encode_int(value, how), INT32)
            else:
                master.mav.param_set_send(ts, tc, name.encode(), float(value), REAL32)
            msg = read_back(master, name)
            if msg is None:
                continue
            got = decode_int(msg.param_value) if kind == "int" else (msg.param_value,)
            if value in got or (kind != "int" and abs(got[0] - value) < 1e-6):
                ok = True
                break
        print(f"  {name} = {value}: {'OK' if ok else 'FAILED'}")
        failed |= not ok
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
