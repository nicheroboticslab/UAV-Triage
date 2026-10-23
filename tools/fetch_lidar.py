#!/usr/bin/env python3
"""
List and pick USGS 3DEP lidar point-cloud tiles (LPC) for the world area.

    python3 tools/fetch_lidar.py               # list lidar projects covering the area
    python3 tools/fetch_lidar.py --project NAME  # write that project's URLs to geo/lidar_urls.txt

Uses the TNM Access API (https://tnmaccess.nationalmap.gov/api/v1/docs).
The area is the same box fetch_geo.py uses (home +/- 1500 m from scenario.json).
"""
import json
import math
import os
import sys
import urllib.parse
import urllib.request
from collections import defaultdict

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
API = "https://tnmaccess.nationalmap.gov/api/v1/products"
M_LAT = 111320.0


def bbox(half=1500.0):
    sc = json.load(open(os.path.join(REPO, "scenario.json")))
    lat0, lon0 = sc["home"]["lat"], sc["home"]["lon"]
    m_lon = M_LAT * math.cos(math.radians(lat0))
    return (lon0 - half / m_lon, lat0 - half / M_LAT, lon0 + half / m_lon, lat0 + half / M_LAT)


def query(box):
    items, offset = [], 0
    while True:
        q = urllib.parse.urlencode({"datasets": "Lidar Point Cloud (LPC)",
                                    "bbox": ",".join(f"{v:.6f}" for v in box),
                                    "max": 200, "offset": offset, "outputFormat": "JSON"})
        req = urllib.request.Request(API + "?" + q, headers={"User-Agent": "UAV-Triage-thesis/1.0"})
        with urllib.request.urlopen(req, timeout=120) as r:
            d = json.loads(r.read())
        batch = d.get("items", [])
        items += batch
        offset += len(batch)
        if not batch or offset >= d.get("total", 0):
            return items, d.get("total")


def project_of(item):
    # Titles look like "USGS Lidar Point Cloud <PROJECT> <TILE>"
    t = item.get("title", "")
    t = t.replace("USGS Lidar Point Cloud", "").strip()
    return t.rsplit(" ", 1)[0] if " " in t else t


def main():
    box = bbox()
    print("bbox W,S,E,N:", ", ".join(f"{v:.6f}" for v in box))
    items, total = query(box)
    print(f"API total: {total}, received: {len(items)}")
    if items:
        print("item fields:", sorted(items[0].keys()))
    groups = defaultdict(list)
    for it in items:
        groups[project_of(it)].append(it)
    if "--project" not in sys.argv:
        print(f"\n{'project':60s} {'tiles':>5s} {'GB':>7s}  published")
        for name, its in sorted(groups.items(), key=lambda kv: max(i.get("publicationDate", "") for i in kv[1]), reverse=True):
            gb = sum(i.get("sizeInBytes") or 0 for i in its) / 1e9
            pub = max(i.get("publicationDate", "") for i in its)
            print(f"{name[:60]:60s} {len(its):5d} {gb:7.2f}  {pub}")
        print("\nPick one: python3 tools/fetch_lidar.py --project NAME")
        return
    name = sys.argv[sys.argv.index("--project") + 1]
    its = groups.get(name)
    if not its:
        sys.exit(f"No project named {name!r}")
    urls = [i.get("downloadLazURL") or i.get("downloadURL") for i in its]
    urls = [u for u in urls if u]
    os.makedirs(os.path.join(REPO, "geo"), exist_ok=True)
    out = os.path.join(REPO, "geo", "lidar_urls.txt")
    open(out, "w").write("\n".join(urls) + "\n")
    meta = {"project": name, "bbox_wsen": box, "tiles": [i.get("title") for i in its], "api": API}
    json.dump(meta, open(os.path.join(REPO, "geo", "lidar_meta.json"), "w"), indent=2)
    print(f"{len(urls)} URLs -> {out}")


if __name__ == "__main__":
    main()
