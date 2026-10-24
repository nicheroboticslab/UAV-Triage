"""
Common time stamp for every logged record (Ben, 24 Oct: "add the current CPU
timestamp for all logged datapoints").

    wall      seconds since 1970-01-01 UTC (time.time()) on this PC
    wall_iso  the same instant as local time with UTC offset, ms resolution
    mono_ns   time.monotonic_ns(): never jumps (NTP, DST); same clock in every
              process on this PC since boot, so it orders records across files

All simulator logs, the portal and the Polar H10 recorder run on the same PC,
so these stamps align every file directly.
"""
import time
from datetime import datetime


def stamp():
    w = time.time()
    return {"wall": w,
            "wall_iso": datetime.fromtimestamp(w).astimezone().isoformat(timespec="milliseconds"),
            "mono_ns": time.monotonic_ns()}


def stamped(rec):
    """Return rec with the stamp fields first; fields already in rec are kept."""
    out = stamp()
    if "wall" in rec:               # keep a time the caller already took; ISO from that same instant
        out["wall"] = rec["wall"]
        out["wall_iso"] = datetime.fromtimestamp(rec["wall"]).astimezone().isoformat(timespec="milliseconds")
    for k in ("wall_iso", "mono_ns"):
        if k in rec:
            out[k] = rec[k]
    out.update({k: v for k, v in rec.items() if k not in out})
    return out
