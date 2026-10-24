#!/usr/bin/env python3
"""
Archive a run (H6): copy every file of one run into ~/uav_dataset/runs/<run_id>/
with a SHA-256 manifest, so each block's data stays tied to its code version,
session, call set, scene positions and hazards.

    python3 tools/archive_run.py --latest      # newest run (called by stop_session.sh)
    python3 tools/archive_run.py --pending     # every run not archived yet (called by the launcher)
    python3 tools/archive_run.py 20261024_093012

A run = files in ~/uav_dataset whose name stamp is within STAMP_TOL_S of the run
id (logs open within a second or two of each other): meta, inaction, battery,
controls, flight, condition, reports, tasks, portal, nofly, h10 ... Polar H10
files are matched by time overlap instead (they span several blocks).
Originals are left in place; the archive is a verified copy.
"""
import glob
import hashlib
import json
import os
import re
import shutil
import sys
import time
from datetime import datetime

DATA = os.path.expanduser(os.environ.get("UAV_DATASET", "~/uav_dataset"))
RUNS = os.path.join(DATA, "runs")
STAMP = re.compile(r"^([a-z0-9]+)_(\d{8}_\d{6})\.(jsonl|json|csv)$")
STAMP_TOL_S = 5


def ts(stamp):
    return datetime.strptime(stamp, "%Y%m%d_%H%M%S").timestamp()


def run_ids():
    """Runs are identified by their inaction log (written by every pilot run)."""
    ids = []
    for p in glob.glob(os.path.join(DATA, "inaction_*.jsonl")):
        m = STAMP.match(os.path.basename(p))
        if m:
            ids.append(m.group(2))
    return sorted(ids)


def files_for(run_id):
    t0 = ts(run_id)
    out = []
    for p in sorted(os.listdir(DATA)):
        m = STAMP.match(p)
        if not m or m.group(1) == "h10":
            continue
        if abs(ts(m.group(2)) - t0) <= STAMP_TOL_S:
            out.append(os.path.join(DATA, p))
    # Polar H10 files: include any whose recording overlaps the run
    end = max([os.path.getmtime(p) for p in out] + [t0])
    for p in glob.glob(os.path.join(DATA, "h10_*.jsonl")):
        m = STAMP.match(os.path.basename(p))
        if m and ts(m.group(2)) <= end and os.path.getmtime(p) >= t0:
            out.append(p)
    return out


def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def archive(run_id, force=False):
    dest = os.path.join(RUNS, run_id)
    if os.path.exists(os.path.join(dest, "MANIFEST.sha256")) and not force:
        print(f"{run_id}: already archived")
        return dest
    files = files_for(run_id)
    if not files:
        print(f"{run_id}: no files")
        return None
    os.makedirs(dest, exist_ok=True)
    lines, summary = [], {"run_id": run_id, "archived_wall": time.time(), "files": []}
    for p in files:
        q = os.path.join(dest, os.path.basename(p))
        shutil.copy2(p, q)
        h1, h2 = sha(p), sha(q)
        if h1 != h2:
            sys.exit(f"{run_id}: copy of {p} does not match the original")
        lines.append(f"{h1}  {os.path.basename(p)}")
        summary["files"].append({"name": os.path.basename(p), "sha256": h1, "bytes": os.path.getsize(p)})
    metas = sorted(glob.glob(os.path.join(dest, "meta_*.json")))
    meta = metas[0] if metas else None
    if meta:
        m = json.load(open(meta))
        summary["git_commit"] = m.get("git", {}).get("commit")
        summary["git_dirty"] = bool(m.get("git", {}).get("status_porcelain"))
        summary["session_block"] = (m.get("session", {}).get("content") or {}).get("block")
    else:
        summary["warning"] = "no meta file (run predates H6 or the pilot failed to write it)"
    json.dump(summary, open(os.path.join(dest, "archive.json"), "w"), indent=1)
    with open(os.path.join(dest, "MANIFEST.sha256"), "w") as f:
        f.write("\n".join(lines) + "\n")
    flag = " (UNCOMMITTED CODE)" if summary.get("git_dirty") else ""
    print(f"{run_id}: {len(files)} files -> {dest}  git {str(summary.get('git_commit'))[:10]}{flag}")
    return dest


def main():
    a = sys.argv[1:]
    if not a:
        sys.exit(__doc__)
    ids = run_ids()
    if a[0] == "--latest":
        if ids:
            archive(ids[-1], force=True)
    elif a[0] == "--pending":
        done = set(os.listdir(RUNS)) if os.path.isdir(RUNS) else set()
        for r in ids:
            if r not in done and ts(r) > ts("20261024_000000"):   # older dev runs are not archived
                archive(r)
    else:
        archive(a[0], force="--force" in a)


if __name__ == "__main__":
    main()
