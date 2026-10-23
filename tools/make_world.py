#!/usr/bin/env python3
"""
Generate the Gazebo world for one call set (E3 / C5 / C2).

    python3 tools/make_world.py BASELINE      # -> worlds/egle_BASELINE.sdf

World frame: Gazebo ENU, origin = PX4 home (PX4 sets the world origin to
PX4_HOME_LAT/LON), so a call at (north, east) is placed at x = east, y = north.
Contents: large ground plane, overcast lighting (no flashing effects), the
EGLe drone (x500 + 45-degree camera) at home, and per-call scene props built
from the outcome code (scenario_code) via TEMPLATES below.
Props use Gazebo Fuel models (OpenRobotics Hatchback, Rescue Randy); the first
launch downloads them (cached in ~/.gz/fuel afterwards).
"""
import json
import math
import os
import sys

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
sys.path.insert(0, os.path.join(REPO, "ros_ws/src/uav_control/uav_control"))
import call_sets  # noqa: E402

FUEL = "https://fuel.gazebosim.org/1.0/OpenRobotics/models"
CAR = f"{FUEL}/Hatchback"
PERSON = f"{FUEL}/Rescue Randy"
PERSON_SIT = f"{FUEL}/Rescue Randy Sitting"


def include(uri, name, x, y, z=0.0, yaw=0.0):
    return (f'    <include><uri>{uri}</uri><name>{name}</name>'
            f'<pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.3f}</pose><static>true</static></include>\n')


def box(name, x, y, z, sx, sy, sz, rgba, yaw=0.0, collide=True):
    col = (f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>'
           if collide else "")
    return (f'    <model name="{name}"><static>true</static><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.3f}</pose>'
            f'<link name="l">{col}<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
            f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual></link></model>\n')


def water(name, x, y, size, depth):
    # translucent flood water patch; top surface at 'depth' metres above ground
    return box(name, x, y, depth / 2, size, size, depth, "0.25 0.32 0.35 0.75", collide=False)


def people(prefix, x, y, n, z=0.0, spacing=1.2, sitting=0):
    out = ""
    for i in range(n):
        uri = PERSON_SIT if i < sitting else PERSON
        out += include(uri, f"{prefix}_p{i}", x + (i - (n - 1) / 2) * spacing, y, z, yaw=1.57)
    return out


def building(prefix, x, y):
    # 20 x 14 x 8 m block; entrance faces -y (south) at (x, y - 7)
    return box(f"{prefix}_bldg", x, y + 7, 4, 20, 14, 8, "0.55 0.52 0.48 1")


# outcome code -> scene builder(prefix, x, y, truth)
def t_S2a(p, x, y, t):   # N people at the entrance
    return building(p, x, y) + water(f"{p}_w", x, y - 4, 30, 0.3) + people(p, x, y - 1.5, t["people"])

def t_S2c(p, x, y, t):   # N people nearby, outside the building
    return building(p, x, y) + water(f"{p}_w", x, y - 4, 30, 0.3) + people(p, x + 14, y - 6, t["people"])

def t_S4b(p, x, y, t):   # bridge submerged with N people on it
    return (box(f"{p}_bridge", x, y, 0.4, 3, 30, 0.4, "0.45 0.40 0.35 1")
            + water(f"{p}_w", x, y, 40, 0.6) + people(p, x, y, t["people"], z=0.6, spacing=2.0))

def t_S5b(p, x, y, t):   # several occupied cars in a lot (occupants visible next to/on cars)
    out = water(f"{p}_w", x, y, 40, 0.3)
    for i in range(6):
        cx, cy = x - 10 + (i % 3) * 8, y - 3 + (i // 3) * 6
        out += include(CAR, f"{p}_car{i}", cx, cy, 0, 1.57)
    for i in range(t["people"]):   # one person beside each of the first N cars
        out += include(PERSON, f"{p}_p{i}", x - 10 + i * 8 + 1.6, y - 3, 0, 1.57)
    return out

TEMPLATES = {"S2a": t_S2a, "S2c": t_S2c, "S4b": t_S4b, "S5b": t_S5b}


# ---------------------------------------------------------------------------
# Geodata layer (C6 imagery ground, OSM buildings and trees). Optional: used
# when geo/geo_meta.json exists (tools/fetch_geo.py).
# ---------------------------------------------------------------------------
GEO = os.path.join(REPO, "geo")
DEFAULT_BLDG_H = 8.0
LEVEL_H = 3.2
HOME_CLEAR_M = 12.0        # drop any building within this distance of the takeoff point


def _num(v):
    try:
        return float(str(v).split()[0].replace("m", ""))
    except (ValueError, IndexError):
        return None


def load_geo(home):
    meta_p = os.path.join(GEO, "geo_meta.json")
    if not os.path.exists(meta_p):
        return None
    meta = json.load(open(meta_p))
    osm = json.load(open(os.path.join(GEO, "osm.json")))["elements"]
    nodes = {el["id"]: (el["lat"], el["lon"]) for el in osm if el["type"] == "node" and "lat" in el}
    m_lon = call_sets.M_LAT * math.cos(math.radians(home["lat"]))

    def en(lat, lon):
        return ((lon - home["lon"]) * m_lon, (lat - home["lat"]) * call_sets.M_LAT)

    buildings, trees, skipped = [], [], {"relation": 0, "home": 0, "degenerate": 0}
    for el in osm:
        tags = el.get("tags", {})
        if el["type"] == "relation" and "building" in tags:
            skipped["relation"] += 1
        elif el["type"] == "way" and "building" in tags:
            pts = [en(*nodes[n]) for n in el.get("nodes", []) if n in nodes]
            if len(pts) > 1 and pts[0] == pts[-1]:
                pts = pts[:-1]
            if len(pts) < 3:
                skipped["degenerate"] += 1
                continue
            if point_in_poly(0.0, 0.0, pts) or min(math.hypot(x, y) for x, y in pts) < HOME_CLEAR_M:
                skipped["home"] += 1
                continue
            h = _num(tags.get("height"))
            if h is None and _num(tags.get("building:levels")) is not None:
                h = _num(tags["building:levels"]) * LEVEL_H
            buildings.append({"id": el["id"], "pts": pts, "h": h or DEFAULT_BLDG_H,
                              "h_source": "height" if _num(tags.get("height")) else
                              ("levels" if _num(tags.get("building:levels")) else "default")})
        elif el["type"] == "node" and tags.get("natural") == "tree" and "lat" in el:
            x, y = en(el["lat"], el["lon"])
            if math.hypot(x, y) >= HOME_CLEAR_M:
                trees.append({"x": x, "y": y, "h": _num(tags.get("height")) or 10.0})
    return {"meta": meta, "buildings": buildings, "trees": trees, "skipped": skipped}


def point_in_poly(x, y, poly):
    inside, j = False, len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def outside_point(x, y, buildings, margin=3.0):
    """If (x, y) is inside a building footprint, return the nearest point just outside its wall."""
    for b in buildings:
        if point_in_poly(x, y, b["pts"]):
            best = None
            pts = b["pts"]
            for i in range(len(pts)):
                (ax, ay), (bx, by) = pts[i], pts[(i + 1) % len(pts)]
                dx, dy = bx - ax, by - ay
                L2 = dx * dx + dy * dy or 1e-9
                t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
                px, py = ax + t * dx, ay + t * dy
                d = math.hypot(px - x, py - y)
                if best is None or d < best[0]:
                    best = (d, px, py)
            d, px, py = best
            ux, uy = (px - x) / (d or 1e-9), (py - y) / (d or 1e-9)
            return px + ux * margin, py + uy * margin, True
    return x, y, False


def ground_tiles(meta):
    out = ""
    for k, t in enumerate(meta["tiles"]):
        cx, cy = t["east_min"] + t["size_m"] / 2, t["north_min"] + t["size_m"] / 2
        img = os.path.join(GEO, t["file"])
        out += (f'    <model name="ground_tile_{k}"><static>true</static><pose>{cx:.1f} {cy:.1f} 0.02 0 0 0</pose>'
                f'<link name="l"><visual name="v"><geometry><plane><normal>0 0 1</normal>'
                f'<size>{t["size_m"]} {t["size_m"]}</size></plane></geometry>'
                f'<material><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>'
                f'<pbr><metal><albedo_map>{img}</albedo_map><roughness>1.0</roughness><metalness>0</metalness></metal></pbr>'
                f'</material></visual></link></model>\n')
    return out


def buildings_model(buildings):
    # One static model; each footprint extruded with an SDF polyline (visual + collision).
    parts = []
    for b in buildings:
        pts = "".join(f"<point>{x:.2f} {y:.2f}</point>" for x, y in b["pts"])
        geom = f"<geometry><polyline>{pts}<height>{b['h']:.1f}</height></polyline></geometry>"
        parts.append(f'<collision name="c{b["id"]}">{geom}</collision>'
                     f'<visual name="v{b["id"]}">{geom}<material><ambient>0.50 0.49 0.47 1</ambient>'
                     f'<diffuse>0.55 0.54 0.52 1</diffuse></material></visual>')
    return ('    <model name="osm_buildings"><static>true</static><link name="l">'
            + "".join(parts) + '</link></model>\n')


def trees_model(trees):
    parts = []
    for k, t in enumerate(trees):
        h = t["h"]
        trunk, crown = 0.4 * h, 0.35 * h
        parts.append(
            f'<collision name="tc{k}"><pose>{t["x"]:.1f} {t["y"]:.1f} {h - crown:.1f} 0 0 0</pose>'
            f'<geometry><sphere><radius>{crown:.1f}</radius></sphere></geometry></collision>'
            f'<visual name="tt{k}"><pose>{t["x"]:.1f} {t["y"]:.1f} {trunk / 2:.1f} 0 0 0</pose>'
            f'<geometry><cylinder><radius>0.25</radius><length>{trunk:.1f}</length></cylinder></geometry>'
            f'<material><diffuse>0.30 0.22 0.15 1</diffuse></material></visual>'
            f'<visual name="tv{k}"><pose>{t["x"]:.1f} {t["y"]:.1f} {h - crown:.1f} 0 0 0</pose>'
            f'<geometry><sphere><radius>{crown:.1f}</radius></sphere></geometry>'
            f'<material><diffuse>0.18 0.32 0.16 1</diffuse></material></visual>')
    return ('    <model name="osm_trees"><static>true</static><link name="l">'
            + "".join(parts) + '</link></model>\n') if parts else ""


def drone():
    # EGLe drone: x500 with a mono camera pitched 45 degrees down, facing forward.
    return ('    <include><uri>model://egle_drone</uri><name>egle_drone</name>'
            '<pose>0 0 0.25 0 0 0</pose></include>\n')


def world(set_id):
    sc = json.load(open(os.path.join(REPO, "scenario.json")))
    cs, calls = call_sets.resolve(set_id, sc["home"], float(sc.get("cruise_alt_m", 10)))
    by_id = {c["call_id"]: c for c in cs["calls"]}
    geo = load_geo(sc["home"])
    global building
    if geo:
        building = lambda p, x, y: ""   # real OSM buildings replace the placeholder box
    body, missing, moved = "", [], []
    for c in calls:
        n, e, _ = c["pos"]
        if geo:
            e2, n2, was_inside = outside_point(e, n, geo["buildings"])
            if was_inside:
                moved.append(c["call_id"])
                e, n = e2, n2
        code = c.get("scenario_code")
        tmpl = TEMPLATES.get(code)
        if tmpl is None:
            missing.append(f'{c["call_id"]}:{code}')
            continue
        body += f'    <!-- {c["call_id"]} {c["location"]} {c["label"]}: {code} -->\n'
        body += tmpl(c["call_id"].lower(), e, n, by_id[c["call_id"]].get("truth", {}))
    name = f"egle_{set_id}"
    sdf = f'''<?xml version="1.0"?>
<!-- Generated by tools/make_world.py from calls/sets/{set_id}.json. Do not edit by hand. -->
<sdf version="1.9">
  <world name="{name}">
    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate></physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"/>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{sc["home"]["lat"]}</latitude_deg><longitude_deg>{sc["home"]["lon"]}</longitude_deg>
      <elevation>{sc["home"].get("alt_m", 0)}</elevation></spherical_coordinates>
    <!-- C2: overcast, dim, no lightning or flashing -->
    <scene><grid>false</grid><ambient>0.30 0.30 0.32 1</ambient><background>0.42 0.44 0.47 1</background>
      <shadows>false</shadows></scene>
    <light name="overcast" type="directional"><pose>0 0 500 0 0 0</pose><cast_shadows>false</cast_shadows>
      <intensity>0.6</intensity><direction>0.1 0.3 -0.95</direction>
      <diffuse>0.55 0.56 0.58 1</diffuse><specular>0.1 0.1 0.1 1</specular></light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>4000 4000</size></plane></geometry></collision>
      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>4000 4000</size></plane></geometry>
        <material><ambient>0.30 0.33 0.28 1</ambient><diffuse>0.30 0.33 0.28 1</diffuse></material></visual>
    </link></model>
{ground_tiles(geo["meta"]) if geo else ""}{buildings_model(geo["buildings"]) if geo else ""}{trees_model(geo["trees"]) if geo else ""}{drone()}{body}  </world>
</sdf>
'''
    out = os.path.join(REPO, "worlds", f"{name}.sdf")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    open(out, "w").write(sdf)
    if geo:
        hs = sorted(b["h"] for b in geo["buildings"])
        src = {k: sum(1 for b in geo["buildings"] if b["h_source"] == k) for k in ("height", "levels", "default")}
        print(f"Geo: {len(geo['meta']['tiles'])} imagery tiles, {len(hs)} buildings "
              f"(height from tag {src['height']}, levels {src['levels']}, default {src['default']}; "
              f"tallest {hs[-1] if hs else 0:.1f} m), {len(geo['trees'])} trees; skipped {geo['skipped']}")
        if moved:
            print("Call props moved outside a building footprint:", ", ".join(moved))
    else:
        print("No geo/ data: flat ground (run tools/fetch_geo.py)")
    return out, name, missing


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path, name, missing = world(sys.argv[1])
    print(path)
    if missing:
        print("WARNING: no template for:", ", ".join(missing), file=sys.stderr)
