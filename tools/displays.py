#!/usr/bin/env python3
"""
Monitor assignment for the participant windows (Ben, 24 Oct: screen 2 = portal,
screen 4 = camera + minimap).

    python3 tools/displays.py              # list monitors with their numbers and roles
    python3 tools/displays.py geom camera  # prints "x y w h" of the camera monitor

Monitors are numbered 1, 2, 3, ... in the order `xrandr --listmonitors` reports
them. displays.json (repo root) maps a role to a monitor number or to an output
name (e.g. "HDMI-1"; more robust if the order ever changes):
    {"portal": 2, "camera": 4}
If the monitor is not connected, geom prints nothing and exits 1; the window then
opens wherever the window manager puts it.
"""
import json
import os
import re
import subprocess
import sys

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
CFG = os.path.join(REPO, "displays.json")
LINE = re.compile(r"^\s*(\d+):\s+\S+\s+(\d+)/\d+x(\d+)/\d+\+(-?\d+)\+(-?\d+)\s+(\S+)")


def monitors():
    try:
        out = subprocess.run(["xrandr", "--listmonitors"], capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return []
    mons = []
    for line in out.splitlines():
        m = LINE.match(line)
        if m:
            i, w, h, x, y, name = m.groups()
            mons.append({"n": int(i) + 1, "name": name, "w": int(w), "h": int(h), "x": int(x), "y": int(y)})
    return mons


def config():
    try:
        return {k: v for k, v in json.load(open(CFG)).items() if not k.startswith("_")}
    except FileNotFoundError:
        return {"portal": 2, "camera": 4}


def find(role):
    want = config().get(role)
    for m in monitors():
        if want == m["n"] or (isinstance(want, str) and want == m["name"]):
            return m
    return None


def main():
    a = sys.argv[1:]
    if a[:1] == ["geom"] and len(a) == 2:
        m = find(a[1])
        if not m:
            sys.exit(1)
        print(m["x"], m["y"], m["w"], m["h"])
        return
    roles = config()
    mons = monitors()
    if not mons:
        sys.exit("xrandr found no monitors (not an X11 session?)")
    for m in mons:
        r = [k for k, v in roles.items() if v in (m["n"], m["name"])]
        print(f'{m["n"]}: {m["name"]:10s} {m["w"]}x{m["h"]}+{m["x"]}+{m["y"]}  {", ".join(r)}')
    for k, v in roles.items():
        if not find(k):
            print(f"WARNING: {k} -> monitor {v} not connected")


if __name__ == "__main__":
    main()
