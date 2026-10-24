#!/usr/bin/env python3
"""
Make flowing flood water look in motion (Ben, 23 Oct): move the foam patches
and debris that tools/make_world.py placed on each flowing-water patch
downstream at a steady speed, wrapping at the end of the patch.

    python3 tools/flow_animator.py BASELINE      # started by tools/start_world.sh

Reads worlds/egle_<SET>_flow.json (written by make_world.py). Moves the items
with Gazebo's /world/<world>/set_pose_vector service (one request per tick for
all items); falls back to /set_pose per item if the vector service is missing.
Slow, continuous motion, no brightness change: nothing flashes (E7).
Positions depend only on elapsed time, so the motion is the same every run.
"""
import json
import math
import os
import sys
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
RATE_HZ = 10.0
TIMEOUT_MS = 500


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def fill(p, name, x, y, z, q):
    p.name = name
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q


def main():
    set_id = sys.argv[1] if len(sys.argv) > 1 else "BASELINE"
    world = f"egle_{set_id}"
    man = json.load(open(os.path.join(REPO, "worlds", f"{world}_flow.json")))
    patches = man["items"]
    if not patches:
        print(f"[flow] {world}: no flowing water; nothing to animate")
        return
    node = Node()
    svc_v, svc_1 = f"/world/{world}/set_pose_vector", f"/world/{world}/set_pose"

    print(f"[flow] waiting for {world} ...", flush=True)
    while True:                                   # wait until the world serves requests
        ok, _ = node.request(svc_1, Pose(), Pose, Boolean, TIMEOUT_MS)
        if ok:
            break
        time.sleep(1.0)
    n = sum(len(p["items"]) for p in patches)
    print(f"[flow] animating {n} items on {len(patches)} flowing patch(es)", flush=True)

    use_vector, t0, dt = True, time.time(), 1.0 / RATE_HZ
    while True:
        t = time.time() - t0
        poses = []
        for p in patches:
            c, s = math.cos(p["flow_yaw"]), math.sin(p["flow_yaw"])
            L = p["length"]
            for it in p["items"]:
                u = (it["u"] + p["speed_ms"] * t + L / 2) % L - L / 2      # wrap along the flow
                x = p["x"] + c * u - s * it["v"]
                y = p["y"] + s * u + c * it["v"]
                r, pi, yw = it["rpy"]
                poses.append((it["name"], x, y, p["z"], quat(r, pi, yw + p["flow_yaw"])))
        if use_vector:
            req = Pose_V()
            for name, x, y, z, q in poses:
                fill(req.pose.add(), name, x, y, z, q)
            ok, _ = node.request(svc_v, req, Pose_V, Boolean, TIMEOUT_MS)
            if not ok:
                print("[flow] set_pose_vector unavailable; using set_pose per item", flush=True)
                use_vector = False
        if not use_vector:
            for name, x, y, z, q in poses:
                req = Pose()
                fill(req, name, x, y, z, q)
                node.request(svc_1, req, Pose, Boolean, TIMEOUT_MS)
        time.sleep(max(0.0, dt - ((time.time() - t0) - t)))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
