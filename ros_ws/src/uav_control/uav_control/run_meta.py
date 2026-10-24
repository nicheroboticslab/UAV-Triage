"""
Run provenance snapshot (H6/H7), written by the pilot script when a run starts.

~/uav_dataset/meta_<run_id>.json holds everything needed to reproduce and score
the block, captured at its start (so later edits to session.json etc. cannot
leak into the archive):
  - session.json (content + sha256), call-set file, scene offsets / scene file,
    no-fly zones, secondary-task file, battery.json
  - world file sha256
  - git commit, branch, uncommitted files and the diff of tracked files
  - host, python version, start time
tools/archive_run.py bundles this with the run's logs.
"""
import hashlib
import json
import os
import platform
import socket
import subprocess
import time

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))


def _read(path):
    try:
        with open(path, "rb") as f:
            raw = f.read()
        return {"path": path, "sha256": hashlib.sha256(raw).hexdigest(), "bytes": len(raw),
                "content": json.loads(raw.decode("utf-8")) if path.endswith(".json") else None}
    except FileNotFoundError:
        return {"path": path, "missing": True}
    except Exception as e:
        return {"path": path, "error": f"{type(e).__name__}: {e}"}


def _sha(path):
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()
    except Exception:
        return None


def _git(*args):
    try:
        return subprocess.run(["git", "-C", REPO, *args], capture_output=True, text=True, timeout=10).stdout
    except Exception as e:
        return f"ERROR {e}"


def write_meta(dataset_dir, run_id, session, call_set_id):
    world = f"egle_{call_set_id}" if call_set_id else None
    diff = _git("diff")
    meta = {
        "type": "run_meta", "run_id": run_id, "written_wall": time.time(),
        "host": socket.gethostname(), "python": platform.python_version(),
        "git": {"commit": _git("rev-parse", "HEAD").strip(),
                "branch": _git("rev-parse", "--abbrev-ref", "HEAD").strip(),
                "status_porcelain": _git("status", "--porcelain").splitlines(),
                "diff_sha256": hashlib.sha256(diff.encode()).hexdigest(),
                "diff": diff if len(diff) < 400_000 else "(too large; see diff_sha256)"},
        "session": _read(session.get("path", os.path.join(REPO, "session.json"))),
        "call_set": _read(os.path.join(REPO, "calls", "sets", f"{call_set_id}.json")) if call_set_id else None,
        "scene_offsets": _read(os.path.join(REPO, "calls", "scene_offsets.json")),
        "scenes": _read(os.path.join(REPO, "worlds", f"{world}_scenes.json")) if world else None,
        "hazards": _read(os.path.join(REPO, "worlds", f"{world}_hazards.json")) if world else None,
        "tasks": _read(os.path.join(REPO, "calls", "tasks", f"{call_set_id}.json")) if call_set_id else None,
        "battery": _read(os.path.join(REPO, "battery.json")),
        "world_sdf_sha256": _sha(os.path.join(REPO, "worlds", f"{world}.sdf")) if world else None,
    }
    path = os.path.join(dataset_dir, f"meta_{run_id}.json")
    with open(path, "w") as f:
        json.dump(meta, f, indent=1)
    return path, meta["git"]["commit"], bool(meta["git"]["status_porcelain"])
