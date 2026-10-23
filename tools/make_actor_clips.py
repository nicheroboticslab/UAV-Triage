#!/usr/bin/env python3
"""
Make faster copies of Gazebo actor animation clips (Ben, 23 Oct: the gesturing
of mobile people was far too slow). SDF has no playback-speed setting, so the
clip's keyframe times are scaled instead.

    python3 tools/make_actor_clips.py            # once; writes models/egle_people/meshes/

Downloads talk_b.dae from the Fuel model Mingfei/actor (the model used in
Gazebo's own actor examples) and writes talk_b_fast.dae with every animation
TIME array divided by SPEED. Mesh, skin and bones are unchanged.
tools/make_world.py uses the fast clip when the file exists.
"""
import os
import sys
import urllib.request
import xml.etree.ElementTree as ET

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
SRC = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes"
OUT = os.path.join(REPO, "models", "egle_people", "meshes")
CLIPS = {"talk_b": 2.5}          # clip -> speed factor
NS = "http://www.collada.org/2005/11/COLLADASchema"


def speed_up(data, speed):
    ET.register_namespace("", NS)
    root = ET.fromstring(data)
    q = lambda t: f"{{{NS}}}{t}"
    n = 0
    for src in root.iter(q("source")):
        params = [p.get("name", "").upper() for p in src.iter(q("param"))]
        arr = src.find(q("float_array"))
        if arr is not None and params == ["TIME"] and arr.text:
            arr.text = " ".join(f"{float(v) / speed:.6g}" for v in arr.text.split())
            n += 1
    if n == 0:
        sys.exit("No TIME arrays found; the clip format is not as expected")
    return ET.tostring(root, encoding="utf-8", xml_declaration=True), n


def main():
    os.makedirs(OUT, exist_ok=True)
    for clip, speed in CLIPS.items():
        req = urllib.request.Request(f"{SRC}/{clip}.dae", headers={"User-Agent": "UAV-Triage-thesis/1.0"})
        data = urllib.request.urlopen(req, timeout=120).read()
        out, n = speed_up(data, speed)
        path = os.path.join(OUT, f"{clip}_fast.dae")
        open(path, "wb").write(out)
        print(f"{clip}: {n} time arrays scaled x{speed} -> {path}")


if __name__ == "__main__":
    main()
