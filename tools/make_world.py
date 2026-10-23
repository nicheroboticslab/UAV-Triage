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
import time

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


# Flood water (decisions 7.8): category -> depth of water above its base.
# Knee (~0.5 m) is the shallow/deep line; depths sit clearly either side.
# Depth is read from what is naturally at the scene: people's legs, car wheels.
WATER_DEPTH_M = {"standing_shallow": 0.15, "standing_deep": 1.00, "flowing": 0.15}   # default level per category
# Four depth levels (Ben, 23 Oct 13:41), measured on an adult standing in it:
# none / ankle (foam ring at the ankles) / waist / neck. A call may set
# truth["depth"] to one of these; otherwise the category default above applies
# (shallow -> ankle, deep -> waist, flowing -> ankle).
WATER_LEVEL_M = {"none": 0.0, "ankle": 0.15, "waist": 1.00, "neck": 1.45}


def depth_for(t):
    lv = t.get("depth")
    if lv is not None:
        if lv not in WATER_LEVEL_M:
            raise ValueError(f"unknown depth level {lv!r} (use {', '.join(WATER_LEVEL_M)})")
        return WATER_LEVEL_M[lv]
    cat = t.get("water")
    return WATER_DEPTH_M.get(cat, 0.0) if cat and cat != "none" else 0.0
WATER_TEX = os.path.join(REPO, "models", "flood_water", "textures")   # tools/make_water_textures.py
WATER_TRANSPARENCY = 0.3      # 70 % opaque (Ben, 23 Oct)
FLOOD_BASE_M = 0.10           # world-wide flood sheet depth; scenes raise the level locally
FLOOD_SIZE_M = 4000.0         # = ground plane
FLOOD_RGBA = "0.40 0.44 0.45 1"   # blue-grey-brown, matches the standing texture
FOAM_RGBA = "0.86 0.83 0.76 1"
FOAM_R = 0.45                 # foam collar radius around a person at the waterline (m)
WAKE_LEN = 1.6                # flowing: wake trail downstream of each person (m)


def water(name, x, y, sx, sy, category, base=0.0, flow_yaw=0.0, depth=None):
    """Flood-water slab from the ground to base + depth(category).
    base = height of the surface the water lies on (e.g. a bridge deck).
    flowing: static streak/foam texture, streaks along flow_yaw (rad, ENU,
    0 = east). No animation, so nothing flickers."""
    if not category or category == "none":
        return ""
    if category not in WATER_DEPTH_M:
        raise ValueError(f"{name}: unknown water category {category!r}")
    top = base + (WATER_DEPTH_M[category] if depth is None else depth)
    if top <= base:
        return ""
    tex = os.path.join(WATER_TEX, "flowing.png" if category == "flowing" else "standing.png")
    return (f'    <model name="{name}"><static>true</static><pose>{x:.2f} {y:.2f} {top / 2:.3f} 0 0 {flow_yaw:.3f}</pose>'
            f'<link name="l"><visual name="v"><cast_shadows>false</cast_shadows>'
            f'<transparency>{WATER_TRANSPARENCY}</transparency>'
            f'<geometry><box><size>{sx} {sy} {top:.3f}</size></box></geometry>'
            f'<material><diffuse>1 1 1 1</diffuse><specular>0.35 0.35 0.35 1</specular>'
            f'<pbr><metal><albedo_map>{tex}</albedo_map><roughness>0.2</roughness><metalness>0</metalness></metal></pbr>'
            f'</material></visual></link></model>\n')


def flood_sheet():
    """World-wide shallow flood (Ben, 23 Oct): everything low is under water;
    each call scene sets its own level on top (shallow / deep / flowing)."""
    return (f'    <model name="flood_sheet"><static>true</static><pose>0 0 {FLOOD_BASE_M / 2:.3f} 0 0 0</pose>'
            f'<link name="l"><visual name="v"><cast_shadows>false</cast_shadows>'
            f'<transparency>{WATER_TRANSPARENCY}</transparency>'
            f'<geometry><box><size>{FLOOD_SIZE_M} {FLOOD_SIZE_M} {FLOOD_BASE_M}</size></box></geometry>'
            f'<material><ambient>{FLOOD_RGBA}</ambient><diffuse>{FLOOD_RGBA}</diffuse>'
            f'<specular>0.35 0.35 0.35 1</specular></material></visual></link></model>\n')


def water_top(category, base=0.0, depth=None):
    if not category or category == "none":
        return None
    d = WATER_DEPTH_M[category] if depth is None else depth
    return base + d if d > 0 else None


def foam(name, x, y, top, flowing=False, flow_yaw=0.0):
    """Light foam collar at the waterline around a person (and a wake trail
    downstream when flowing): shows at a glance that they stand in water and
    where the water reaches on their legs."""
    out = (f'    <model name="{name}"><static>true</static><pose>{x:.2f} {y:.2f} {top + 0.01:.3f} 0 0 0</pose>'
           f'<link name="l"><visual name="v"><cast_shadows>false</cast_shadows>'
           f'<geometry><cylinder><radius>{FOAM_R}</radius><length>0.02</length></cylinder></geometry>'
           f'<material><ambient>{FOAM_RGBA}</ambient><diffuse>{FOAM_RGBA}</diffuse></material></visual></link></model>\n')
    if flowing:
        wx = x + math.cos(flow_yaw) * (FOAM_R + WAKE_LEN / 2)
        wy = y + math.sin(flow_yaw) * (FOAM_R + WAKE_LEN / 2)
        out += box(f"{name}_wake", wx, wy, top + 0.01, WAKE_LEN, 0.35, 0.02, FOAM_RGBA, yaw=flow_yaw, collide=False)
    return out


def people(prefix, x, y, n, z=0.0, spacing=1.2, sitting=0, water=None, flow_yaw=0.0):
    """water = (category, base): people standing in it get a foam collar."""
    out = ""
    top = water_top(*water) if water else None
    for i in range(n):
        uri = PERSON_SIT if i < sitting else PERSON
        px = x + (i - (n - 1) / 2) * spacing
        out += include(uri, f"{prefix}_p{i}", px, y, z, yaw=1.57)
        if top is not None and top > z:
            out += foam(f"{prefix}_p{i}_foam", px, y, top, water[0] == "flowing", flow_yaw)
    return out


def building(prefix, x, y):
    # 20 x 14 x 8 m block; entrance faces -y (south) at (x, y - 7)
    return box(f"{prefix}_bldg", x, y + 7, 4, 20, 14, 8, "0.55 0.52 0.48 1")


# outcome code -> scene builder(prefix, x, y, truth)
def t_S2a(p, x, y, t):   # N people at the entrance, standing in the water
    return (building(p, x, y) + water(f"{p}_w", x, y - 8, 34, 16, t["water"], depth=depth_for(t))
            + people(p, x, y - 1.5, t["people"], water=(t["water"], 0.0, depth_for(t))))

def t_S2c(p, x, y, t):   # N people nearby, outside the building, standing in the water
    return (building(p, x, y) + water(f"{p}_w", x + 4, y - 8, 40, 16, t["water"], depth=depth_for(t))
            + people(p, x + 14, y - 6, t["people"], water=(t["water"], 0.0, depth_for(t))))

def t_S4b(p, x, y, t):   # bridge awash with N people on it; channel flows +x, bridge runs along y
    deck, length, width = 0.6, 28.0, 3.0
    cat = t["water"]
    base = deck if cat == "flowing" else 0.0
    rail = "0.75 0.75 0.72 1"
    out = box(f"{p}_bridge", x, y, deck / 2, width, length, deck, "0.45 0.40 0.35 1")
    for side in (-1, 1):                              # railings stand well above the water
        rx = x + side * (width / 2 - 0.05)
        out += box(f"{p}_rail{side + 1}", rx, y, deck + 1.1, 0.08, length, 0.08, rail, collide=False)
        for k in range(int(length // 2) + 1):
            out += box(f"{p}_post{side + 1}_{k}", rx, y - length / 2 + 2 * k, deck + 0.55, 0.1, 0.1, 1.1, rail, collide=False)
    for end in (-1, 1):                               # banks / abutments, dry, above the water
        out += box(f"{p}_bank{end + 1}", x, y + end * (length / 2 + 1), 0.6, 12, 6, 1.2, "0.42 0.40 0.34 1")
    dep = depth_for(t)
    out += water(f"{p}_w", x, y, 60, length - 6, cat, base=base, depth=dep)      # channel between the banks
    if cat == "flowing":                              # foam piled against the upstream side of the deck
        out += box(f"{p}_pile", x - width / 2 - 0.4, y, water_top(cat, base, dep) + 0.01, 0.8, length - 6, 0.02,
                   FOAM_RGBA, collide=False)
    top = water_top(cat, base, dep)
    n = t["people"]
    for i in range(n):                                # people spaced along the deck
        py = y + (i - (n - 1) / 2) * 2.0
        out += include(PERSON, f"{p}_p{i}", x, py, deck, yaw=0.0)
        if top is not None and top > deck:
            out += foam(f"{p}_p{i}_foam", x, py, top, cat == "flowing", 0.0)
    return out

def t_S5b(p, x, y, t):   # several cars in a lot, people beside some; depth readable on the wheels
    out = water(f"{p}_w", x, y, 40, 24, t["water"], depth=depth_for(t))
    for i in range(6):
        cx, cy = x - 10 + (i % 3) * 8, y - 3 + (i // 3) * 6
        out += include(CAR, f"{p}_car{i}", cx, cy, 0, 1.57)
    top = water_top(t["water"], 0.0, depth_for(t))
    for i in range(t["people"]):   # one person beside each of the first N cars
        px, py = x - 10 + i * 8 + 1.6, y - 3
        out += include(PERSON, f"{p}_p{i}", px, py, 0, 1.57)
        if top is not None:
            out += foam(f"{p}_p{i}_foam", px, py, top, t["water"] == "flowing")
    return out

TEMPLATES = {"S2a": t_S2a, "S2c": t_S2c, "S4b": t_S4b, "S5b": t_S5b}
# Scene footprint (xmin, xmax, ymin, ymax) relative to the anchor, used to keep
# the whole scene clear of real (OSM) buildings. Placeholder buildings excluded.
EXTENT = {"S2a": (-17, 17, -16, 0), "S2c": (-16, 24, -16, 0),
          "S4b": (-30, 30, -18, 18), "S5b": (-20, 20, -12, 12)}
PLACE_MAX_M = 120.0
# Recorded scene offsets (committed): "<location>/<scenario_code>" -> offset of
# the scene from the call pin, metres east/north. Recorded entries are always
# reused, so every build and every participant gets the same scene position.
# New entries are found once (place_clear) and appended. Edit by hand if a
# recorded spot turns out to be inside a building (the build warns).
OFFSETS_FILE = os.path.join(REPO, "calls", "scene_offsets.json")


def load_offsets():
    try:
        return json.load(open(OFFSETS_FILE))
    except FileNotFoundError:
        return {"_help": "Scene offset from the call pin (m east, m north), per location/scenario. "
                         "Written by tools/make_world.py; reused on every build. Edit by hand if needed."}


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


def _bbox(pts):
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), max(xs), min(ys), max(ys)


def footprint_clear(x, y, ext, buildings, step=2.0):
    x0, x1, y0, y1 = x + ext[0], x + ext[1], y + ext[2], y + ext[3]
    for b in buildings:
        bx0, bx1, by0, by1 = b.setdefault("_bbox", _bbox(b["pts"]))
        if bx1 < x0 or bx0 > x1 or by1 < y0 or by0 > y1:
            continue
        if any(x0 <= px <= x1 and y0 <= py <= y1 for px, py in b["pts"]):
            return False
        gx = x0
        while gx <= x1 + 1e-6:
            gy = y0
            while gy <= y1 + 1e-6:
                if point_in_poly(gx, gy, b["pts"]):
                    return False
                gy += step
            gx += step
    return True


def place_clear(x, y, ext, buildings):
    """Nearest anchor (spiral search, 3 m rings, 15 deg) at which the whole
    scene footprint is clear of buildings. Returns (x, y, moved_m) or None."""
    r = 0.0
    while r <= PLACE_MAX_M:
        angles = [0] if r == 0 else range(0, 360, 15)
        for a in angles:
            cx, cy = x + r * math.cos(math.radians(a)), y + r * math.sin(math.radians(a))
            if footprint_clear(cx, cy, ext, buildings):
                return cx, cy, r
        r += 3.0
    return None


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
    offsets, new_offsets = load_offsets(), []
    for c in calls:
        n, e, _ = c["pos"]
        code = c.get("scenario_code")
        key = f'{c["location"]}/{code}'
        if key in offsets:
            de, dn = offsets[key]["east_m"], offsets[key]["north_m"]
            e, n = e + de, n + dn
            if geo and code in EXTENT and not footprint_clear(e, n, EXTENT[code], geo["buildings"]):
                print(f"WARNING {c['call_id']} {key}: recorded offset overlaps a building; edit calls/scene_offsets.json")
            if de or dn:
                moved.append(f"{c['call_id']} {math.hypot(de, dn):.0f} m (recorded)")
        elif geo and code in EXTENT:
            res = place_clear(e, n, EXTENT[code], geo["buildings"])
            if res is None:
                print(f"WARNING {c['call_id']}: no building-free spot within {PLACE_MAX_M:.0f} m; scene may overlap a building")
                res = (e, n, 0.0)
            offsets[key] = {"east_m": round(res[0] - e, 1), "north_m": round(res[1] - n, 1),
                            "found": time.strftime("%Y-%m-%d"), "method": "place_clear"}
            new_offsets.append(key)
            if res[2] > 0:
                moved.append(f"{c['call_id']} {res[2]:.0f} m (new)")
            e, n = e + offsets[key]["east_m"], n + offsets[key]["north_m"]
        elif geo:
            e2, n2, was_inside = outside_point(e, n, geo["buildings"])
            if was_inside:
                moved.append(c["call_id"])
                e, n = e2, n2
        c["scene_en"] = (round(e, 1), round(n, 1))
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
{ground_tiles(geo["meta"]) if geo else ""}{flood_sheet()}{buildings_model(geo["buildings"]) if geo else ""}{trees_model(geo["trees"]) if geo else ""}{drone()}{body}  </world>
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
            print("Scenes moved clear of buildings (from the call pin):", ", ".join(moved))
    else:
        print("No geo/ data: flat ground (run tools/fetch_geo.py)")
    if new_offsets:
        json.dump(offsets, open(OFFSETS_FILE, "w"), indent=2, sort_keys=True)
        print(f"Recorded new scene offsets in calls/scene_offsets.json: {', '.join(new_offsets)}")
    scenes = {c["call_id"]: {"location": c["location"], "pin_en": [round(c["pos"][1], 1), round(c["pos"][0], 1)],
                             "scene_en": list(c.get("scene_en", (c["pos"][1], c["pos"][0])))} for c in calls}
    json.dump(scenes, open(os.path.join(REPO, "worlds", f"{name}_scenes.json"), "w"), indent=2)
    return out, name, missing


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path, name, missing = world(sys.argv[1])
    print(path)
    if missing:
        print("WARNING: no template for:", ", ".join(missing), file=sys.stderr)
