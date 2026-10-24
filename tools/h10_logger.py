#!/usr/bin/env python3
"""
Polar H10 recorder (I1/I2): heart rate + RR intervals (and optionally raw ECG)
over Bluetooth LE, time-stamped on the simulator PC's clock, with session markers.

Run once per participant, in its own terminal, before the first block; leave it
running through the resting baseline and all blocks; stop it with Ctrl+C at the
end. stop_session.sh does not stop it (it spans several blocks); archive_run.py
copies the file into every run it overlaps.

    pip install bleak                       # once (bleakheart too, for --ecg)
    python3 tools/h10_logger.py --scan      # list nearby Polar devices
    python3 tools/h10_logger.py --participant P07                 # first "Polar H10 ..." found
    python3 tools/h10_logger.py --participant P07 --address A0:9E:1A:xx:xx:xx
    python3 tools/h10_logger.py --participant P07 --baseline 300  # 5-min seated rest first (§9.4)
    python3 tools/h10_logger.py --participant P07 --ecg           # also raw ECG, 130 Hz (needs bleakheart)

Output: ~/uav_dataset/h10_<YYYYmmdd_HHMMSS>.jsonl, one JSON object per line:
  {"type":"config", ...}                       device, participant, options
  {"type":"hr", "wall":..., "mono_ns":..., "hr_bpm":..., "rr_1024":[...], "rr_ms":[...], "contact":...}
        one per BLE notification (about 1 per second); rr_1024 = raw RR in 1/1024 s
        exactly as sent (no rounding), rr_ms = the same in milliseconds
  {"type":"ecg", "wall":..., "sensor_t_ns":..., "uV":[...]}   ECG frames (--ecg); sensor_t_ns
        is the H10 time stamp of the LAST sample, converted to epoch by one offset
        taken at the first frame (bleakheart)
  {"type":"marker", "event":..., ...}           baseline_start/end, block_start (START signal),
        run_id, egle_state changes (takeover / handback), block_end, note
  {"type":"status", ...}                        connect / disconnect / battery level
Wall times are time.time() on this PC, the same clock the pilot script and the
portal log with, so physiological data align with every other log directly.

Markers come from the files the simulator already writes: /tmp/uav_start_signal
(START), /tmp/uav_dashboard_state.json (run id, EGLe state, block end).
Type a line + Enter in this terminal to add a free-text "note" marker.
"""
import argparse
import asyncio
import json
import os
import sys
import time
from datetime import datetime

DATA = os.path.expanduser(os.environ.get("UAV_DATASET", "~/uav_dataset"))
HR_CHAR = "00002a37-0000-1000-8000-00805f9b34fb"
BATT_CHAR = "00002a19-0000-1000-8000-00805f9b34fb"
START_FILE = "/tmp/uav_start_signal"
STATE_FILE = "/tmp/uav_dashboard_state.json"


def parse_hr(data: bytes):
    """Bluetooth SIG Heart Rate Measurement (0x2A37). Returns hr, rr in 1/1024 s,
    energy (kJ) and skin contact (None when the sensor does not report it)."""
    flags = data[0]
    i = 1
    if flags & 0x01:
        hr = int.from_bytes(data[i:i + 2], "little"); i += 2
    else:
        hr = data[i]; i += 1
    contact = bool(flags & 0x02) if flags & 0x04 else None
    energy = None
    if flags & 0x08:
        energy = int.from_bytes(data[i:i + 2], "little"); i += 2
    rr = []
    if flags & 0x10:
        while i + 1 < len(data):
            rr.append(int.from_bytes(data[i:i + 2], "little")); i += 2
    return {"hr_bpm": hr, "rr_1024": rr, "rr_ms": [round(v * 1000.0 / 1024.0, 3) for v in rr],
            "energy_kj": energy, "contact": contact}


class Recorder:
    def __init__(self, path, header):
        self.f = open(path, "a", buffering=1)
        self.path = path
        self.n_hr = 0
        self.n_ecg = 0
        self.last_hr = None
        self.write({"type": "config", **header})

    def write(self, rec):
        w = time.time()
        rec = {"wall": rec.pop("wall", w),
               "wall_iso": datetime.fromtimestamp(w).astimezone().isoformat(timespec="milliseconds"),
               "mono_ns": rec.pop("mono_ns", time.monotonic_ns()), **rec}
        self.f.write(json.dumps(rec) + "\n")

    def marker(self, event, **kw):
        self.write({"type": "marker", "event": event, **kw})
        print(f"[h10] {datetime.now():%H:%M:%S} marker {event} {kw if kw else ''}", flush=True)


async def watch_markers(rec: Recorder, stop: asyncio.Event):
    """Poll the simulator's signal/state files and log changes as markers."""
    start_seen = os.path.exists(START_FILE)
    run_id = egle = None
    ended = None
    while not stop.is_set():
        s = os.path.exists(START_FILE)
        if s and not start_seen:
            rec.marker("block_start", start_file_mtime=os.path.getmtime(START_FILE))
        if not s and start_seen:
            rec.marker("start_signal_cleared")
        start_seen = s
        try:
            with open(STATE_FILE) as f:
                st = json.load(f)
            if time.time() - st.get("timestamp", 0) < 5:
                if st.get("run_id") and st["run_id"] != run_id:
                    run_id = st["run_id"]
                    rec.marker("run_id", run_id=run_id, call_set=st.get("call_set"),
                               flight_start_wall=st.get("flight_start_wall"))
                if st.get("egle_state") != egle:
                    rec.marker("egle_state", state=st.get("egle_state"), previous=egle, run_id=run_id)
                    egle = st.get("egle_state")
                if bool(st.get("block_ended")) != bool(ended):
                    ended = bool(st.get("block_ended"))
                    if ended:
                        rec.marker("block_end", run_id=run_id)
        except Exception:
            pass
        await asyncio.sleep(0.2)


async def watch_notes(rec: Recorder, stop: asyncio.Event):
    loop = asyncio.get_running_loop()
    while not stop.is_set():
        line = await loop.run_in_executor(None, sys.stdin.readline)
        if not line:
            return
        line = line.strip()
        if line:
            rec.marker("note", text=line)


async def find_device(name_prefix, address, timeout=15.0):
    from bleak import BleakScanner
    if address:
        return await BleakScanner.find_device_by_address(address, timeout=timeout)
    return await BleakScanner.find_device_by_filter(
        lambda d, adv: bool(d.name) and d.name.startswith(name_prefix), timeout=timeout)


async def baseline(rec: Recorder, seconds: int):
    rec.marker("baseline_start", duration_s=seconds)
    print(f"[h10] Seated rest baseline: {seconds} s. Participant sits still, eyes open, no talking.", flush=True)
    t_end = time.time() + seconds
    while time.time() < t_end:
        left = int(t_end - time.time())
        if left % 30 == 0:
            hr = rec.last_hr
            print(f"[h10]   {left:4d} s left   HR {hr if hr is not None else '--'} bpm", flush=True)
        await asyncio.sleep(1.0)
    rec.marker("baseline_end")
    print("[h10] Baseline done. Recording continues.", flush=True)


async def run(args):
    from bleak import BleakClient
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(DATA, exist_ok=True)
    path = os.path.join(DATA, f"h10_{stamp}.jsonl")
    stop = asyncio.Event()
    rec = None
    baseline_done = args.baseline <= 0
    tasks = []
    while not stop.is_set():
        print(f"[h10] Looking for {args.address or args.name + '*'} ...", flush=True)
        dev = await find_device(args.name, args.address)
        if dev is None:
            print("[h10] Not found. Is the strap worn (it only advertises with skin contact) and not "
                  "connected to a phone? Retrying.", flush=True)
            await asyncio.sleep(3)
            continue
        if rec is None:
            rec = Recorder(path, {"participant": args.participant, "device_name": dev.name,
                                  "address": dev.address, "ecg": args.ecg, "baseline_s": args.baseline,
                                  "host": os.uname().nodename, "note": "wall = time.time() on this PC"})
            print(f"[h10] Recording to {path}", flush=True)
            tasks.append(asyncio.create_task(watch_markers(rec, stop)))
            if sys.stdin.isatty():
                tasks.append(asyncio.create_task(watch_notes(rec, stop)))
        disconnected = asyncio.Event()
        try:
            async with BleakClient(dev, disconnected_callback=lambda c: disconnected.set()) as client:
                rec.write({"type": "status", "event": "connected", "address": dev.address})
                print(f"[h10] Connected to {dev.name} ({dev.address})", flush=True)
                try:
                    b = await client.read_gatt_char(BATT_CHAR)
                    rec.write({"type": "status", "event": "battery", "pct": int(b[0])})
                    print(f"[h10] Strap battery {int(b[0])} %", flush=True)
                except Exception:
                    pass

                def on_hr(_, data):
                    p = parse_hr(bytes(data))
                    rec.write({"type": "hr", **p})
                    rec.n_hr += 1
                    rec.last_hr = p["hr_bpm"]
                    if p["contact"] is False:
                        rec.write({"type": "status", "event": "no_skin_contact"})

                await client.start_notify(HR_CHAR, on_hr)
                if args.ecg:
                    from bleakheart import PolarMeasurementData

                    def on_ecg(frame):
                        kind, t_ns, payload = frame
                        if kind == "ECG":
                            rec.write({"type": "ecg", "sensor_t_ns": t_ns, "uV": payload})
                            rec.n_ecg += 1
                    pmd = PolarMeasurementData(client, callback=on_ecg)
                    err, msg, _ = await pmd.start_streaming("ECG")
                    rec.write({"type": "status", "event": "ecg_start", "code": err, "msg": msg})
                    print(f"[h10] ECG stream: {msg}", flush=True)
                if not baseline_done:
                    baseline_done = True
                    tasks.append(asyncio.create_task(baseline(rec, args.baseline)))
                last_print = 0
                while not disconnected.is_set():
                    await asyncio.sleep(1.0)
                    if time.time() - last_print >= 30:
                        last_print = time.time()
                        print(f"[h10] {datetime.now():%H:%M:%S}  HR {rec.last_hr} bpm  "
                              f"({rec.n_hr} HR frames{f', {rec.n_ecg} ECG frames' if args.ecg else ''})", flush=True)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            print(f"[h10] Connection error: {type(e).__name__}: {e}", flush=True)
        if rec:
            rec.write({"type": "status", "event": "disconnected"})
        print("[h10] Disconnected - reconnecting.", flush=True)
        await asyncio.sleep(1)


async def scan():
    from bleak import BleakScanner
    print("Scanning 10 s ...")
    for d in await BleakScanner.discover(timeout=10.0):
        if d.name and "polar" in d.name.lower():
            print(f"  {d.address}  {d.name}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--participant", default=None)
    ap.add_argument("--name", default="Polar H10", help="device name prefix (default 'Polar H10')")
    ap.add_argument("--address", default=None)
    ap.add_argument("--baseline", type=int, default=0, help="seconds of seated rest baseline at the start")
    ap.add_argument("--ecg", action="store_true", help="also stream raw ECG (130 Hz; pip install bleakheart)")
    ap.add_argument("--scan", action="store_true")
    a = ap.parse_args()
    try:
        asyncio.run(scan() if a.scan else run(a))
    except KeyboardInterrupt:
        print("\n[h10] Stopped.")


if __name__ == "__main__":
    main()
