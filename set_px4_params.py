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
