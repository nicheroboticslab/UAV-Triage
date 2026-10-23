#!/usr/bin/env python3
"""
Download the geodata for the Gazebo world (C6 ground imagery, 3D buildings/trees).
Run once on a machine with internet; results go to ~/UAV-Triage/geo/.

    python3 tools/fetch_geo.py            # 3 x 3 km around home (scenario.json)
    python3 tools/fetch_geo.py --half 2000  # half-width in metres (default 1500)

Imagery: USDA NAIP via the USGS National Map ImageServer (exportImage), requested
in EPSG:4326 so pixels map linearly to latitude/longitude, the same linear
approximation the simulator uses for east/north metres from home.
  https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPImagery/ImageServer
Buildings and trees: OpenStreetMap via the Overpass API (data (c) OpenStreetMap
contributors, ODbL; attribution required).
"""
import json
import math
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
OUT = os.path.join(REPO, "geo")
NAIP = "https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPImagery/ImageServer"
OVERPASS = "https://overpass-api.de/api/interpreter"
M_LAT = 111320.0
TILE_M = 1000.0      # each imagery tile covers 1 km x 1 km
TILE_PX = 2048       # pixels north-south per tile (~0.49 m per pixel)


def get(url, data=None, timeout=180, tries=6):
    """GET/POST with retries: USGS and Overpass return transient 5xx/429 errors."""
    for k in range(tries):
        try:
            req = urllib.request.Request(url, data=data, headers={"User-Agent": "UAV-Triage-thesis/1.0"})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.read()
        except urllib.error.HTTPError as e:
            if e.code not in (429, 500, 502, 503, 504) or k == tries - 1:
                raise
            wait = 5 * (k + 1)
            print(f"    server error {e.code}, retrying in {wait} s ({k + 1}/{tries - 1})")
            time.sleep(wait)
        except (urllib.error.URLError, TimeoutError) as e:
            if k == tries - 1:
                raise
            print(f"    network error ({e}), retrying in 5 s")
            time.sleep(5)


def main():
    half = float(sys.argv[sys.argv.index("--half") + 1]) if "--half" in sys.argv else 1500.0
    sc = json.load(open(os.path.join(REPO, "scenario.json")))
    lat0, lon0 = sc["home"]["lat"], sc["home"]["lon"]
    m_lon = M_LAT * math.cos(math.radians(lat0))
    os.makedirs(os.path.join(OUT, "naip"), exist_ok=True)

    # ---- NAIP tiles ----
    n = int(round(2 * half / TILE_M))
    tiles = []
    info = json.loads(get(NAIP + "?f=json"))
    print("NAIP service:", info.get("name"), "| copyright:", info.get("copyrightText", "")[:120])
    for i in range(n):          # i: east index
        for j in range(n):      # j: north index
            e0, n0 = -half + i * TILE_M, -half + j * TILE_M
            w, s = lon0 + e0 / m_lon, lat0 + n0 / M_LAT
            e, nn = lon0 + (e0 + TILE_M) / m_lon, lat0 + (n0 + TILE_M) / M_LAT
            # The pixel aspect must match the bbox aspect in degrees, otherwise the
            # server widens the bbox to fit the image (seen: +35 % north-south).
            # TILE_PX pixels north-south; more east-west keeps the same ground resolution.
            px_w, px_h = int(round(TILE_PX * (e - w) / (nn - s))), TILE_PX
            if px_w > info.get("maxImageWidth", px_w) or px_h > info.get("maxImageHeight", px_h):
                sys.exit(f"Tile {px_w}x{px_h} px exceeds the server limit; lower TILE_PX")
            params = {"bbox": f"{w},{s},{e},{nn}", "bboxSR": 4326, "imageSR": 4326,
                      "size": f"{px_w},{px_h}", "format": "jpg",
                      "interpolation": "RSP_BilinearInterpolation"}
            name = f"naip_e{int(e0)}_n{int(n0)}.jpg"
            path = os.path.join(OUT, "naip", name)
            if not os.path.exists(path):
                ext = json.loads(get(NAIP + "/exportImage?" + urllib.parse.urlencode({**params, "f": "json"})))["extent"]
                err = max(abs(ext["xmin"] - w), abs(ext["ymin"] - s), abs(ext["xmax"] - e), abs(ext["ymax"] - nn))
                if err * M_LAT > 1.0:
                    sys.exit(f"NAIP extent differs from request by {err * M_LAT:.1f} m for {name}: {ext}")
                img = get(NAIP + "/exportImage?" + urllib.parse.urlencode({**params, "f": "image"}))
                if img[:3] != b"\xff\xd8\xff":
                    sys.exit(f"NAIP returned non-JPEG for {name}: {img[:200]!r}")
                open(path, "wb").write(img)
                time.sleep(0.5)
            tiles.append({"file": f"naip/{name}", "east_min": e0, "north_min": n0, "size_m": TILE_M,
                          "bbox_lonlat": [w, s, e, nn], "px": [px_w, px_h]})
            print("  tile", name)

    # ---- OSM buildings and trees ----
    s, w = lat0 - half / M_LAT, lon0 - half / m_lon
    nn, e = lat0 + half / M_LAT, lon0 + half / m_lon
    query = f"""[out:json][timeout:180];
(way["building"]({s},{w},{nn},{e});
 relation["building"]({s},{w},{nn},{e});
 node["natural"="tree"]({s},{w},{nn},{e});
 way["natural"="wood"]({s},{w},{nn},{e});
 way["landuse"="forest"]({s},{w},{nn},{e}););
out body; >; out skel qt;"""
    osm = get(OVERPASS, data=urllib.parse.urlencode({"data": query}).encode())
    open(os.path.join(OUT, "osm.json"), "wb").write(osm)
    nb = sum(1 for el in json.loads(osm)["elements"] if el.get("tags", {}).get("building"))
    print(f"OSM: {nb} buildings saved")

    meta = {"home": {"lat": lat0, "lon": lon0}, "half_m": half, "tiles": tiles,
            "naip_service": NAIP, "naip_copyright": info.get("copyrightText"),
            "osm_attribution": "(c) OpenStreetMap contributors, ODbL",
            "fetched_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    json.dump(meta, open(os.path.join(OUT, "geo_meta.json"), "w"), indent=2)
    print("Done:", OUT)


if __name__ == "__main__":
    main()
