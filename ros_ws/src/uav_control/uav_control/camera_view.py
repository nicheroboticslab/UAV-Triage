#!/usr/bin/env python3
"""
EGLe camera window for the projector (E3, C2, E7, E1).

Shows the drone's gimbal camera from Gazebo full-screen, with
  - the rain look (C2): static darkening plus a fixed, low-contrast rain overlay;
  - a status strip (E1): EGLe state (who is flying), the agent's current decision
    as a fixed label while it flies (intent + target call, never free-text
    reasoning), battery %, flight time; from /tmp/uav_dashboard_state.json;
  - a neutral block-end screen once the block has ended and the drone landed
    ("Block N complete / Please complete the questionnaire"; no score);
  - a minimap inset (layout option A, Ben 23 Oct): NAIP satellite imagery of
    the world (geo/naip), call pins (C1..C5, from worlds/egle_<SET>_scenes.json),
    home, no-fly zones (dark-red, radar-style; worlds/egle_<SET>_hazards.json),
    the drone with its heading and a track trail, north-up, scale bar.
    D-pad up / down on the controller zooms in / out.
  - Condition (vehicle damage, G1) next to the battery; red while inside a no-fly zone.
E7: nothing flashes; state changes are colour/text changes only.

Reads Gazebo directly (gz-transport13 Python bindings; no ROS bridge, no OpenCV):
the camera image and the drone pose (/world/<world>/dynamic_pose/info).

    python3 camera_view.py                 # finds the camera topic automatically
    python3 camera_view.py --fullscreen    # for the projector
    python3 camera_view.py --topic /world/.../image
"""
import json
import math
import os
import random
import sys
import threading
import time
from collections import deque

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
import pygame  # noqa: E402
try:
    import js_joystick  # noqa: E402  (focus-independent D-pad zoom, Ben 24 Oct)
except ImportError:
    js_joystick = None
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
from gz.transport13 import Node  # noqa: E402

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
STATE_FILE = "/tmp/uav_dashboard_state.json"
WINDOW_TITLE = "EGLe Camera"
DRONE = "egle_drone"

DARKEN_ALPHA = 70        # 0-255, overall dimming for overcast/rain
RAIN_STREAKS = 900
RAIN_ALPHA = 38          # low contrast

# Minimap
MAP_FRAC = 0.42          # inset side as a fraction of the window height
MAP_SPANS_M = [500, 1000, 2000, 3000]    # zoom levels: metres across the inset
MAP_DEFAULT = 2                          # 2000 m across = 1 km radius around the drone (Ben, 24 Oct)
DPAD_UP, DPAD_DOWN = 13, 14              # Ben's pad (controller_input.py --detect)
TRAIL_DT_S, TRAIL_MAX = 0.5, 720         # trail point every 0.5 s, last 6 min
MAP_RGB = {"pin": (255, 196, 0), "pin_txt": (20, 20, 20), "home": (255, 255, 255),
           "drone": (0, 220, 255), "trail": (0, 200, 255), "frame": (210, 210, 210),
           "bg": (25, 28, 32), "nfz_fill": (130, 0, 0, 115), "nfz_rim": (200, 25, 25),
           "nfz_txt": (255, 170, 160)}

STATE_RGB = {"MANUAL": (80, 220, 100), "AUTONOMOUS": (60, 200, 255), "RETURNING": (255, 200, 60),
             "HOLD": (220, 120, 255), "LANDED": (200, 200, 200)}
# Agent decision shown as a fixed label (Experiment Design rec. 1, 24 Oct): the
# selected intent and its target, never the LLM's free-text reasoning.
INTENT_LABEL = {"HOLD_SAFE": "Hold position", "CONTINUE_ASSIGNED_TASK": "Continue current call",
                "SEARCH_NEXT_PRIORITY": "Go to next call", "INSPECT_TARGET_REPORT": "Inspect call",
                "COMMS_REACQUIRE": "Reacquire link", "RETURN_TO_BASE": "Return home",
                "ABORT_AND_SAFE": "Land now"}
STATE_TEXT = {"MANUAL": "YOU ARE FLYING", "AUTONOMOUS": "AGENT IS FLYING", "RETURNING": "RETURNING HOME",
              "HOLD": "HOLDING", "LANDED": "LANDED"}


# ---------------------------------------------------------------- data feeds
class Latest:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None   # (w, h, bytes)
        self.pose = None    # (east, north, yaw_enu)

    def image_cb(self, msg):
        with self.lock:
            self.frame = (msg.width, msg.height, bytes(msg.data))

    def pose_cb(self, msg):
        for p in msg.pose:
            if p.name == DRONE:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                with self.lock:
                    self.pose = (p.position.x, p.position.y, yaw)
                return


def find_topic(node, timeout=30):
    end = time.time() + timeout
    while time.time() < end:
        for t in node.topic_list():
            if DRONE in t and t.endswith("/image"):
                return t
        time.sleep(0.5)
    return None


def read_state():
    try:
        with open(STATE_FILE) as f:
            return json.load(f)
    except Exception:
        return {}


# ---------------------------------------------------------------- rain look
def rain_overlay(size):
    rnd = random.Random(20261023)            # fixed seed: identical every run
    surf = pygame.Surface(size, pygame.SRCALPHA)
    w, h = size
    for _ in range(RAIN_STREAKS):
        x, y = rnd.randrange(w), rnd.randrange(h)
        ln = rnd.randrange(8, 22)
        pygame.draw.line(surf, (200, 205, 215, RAIN_ALPHA), (x, y), (x - 2, y + ln), 1)
    dark = pygame.Surface(size, pygame.SRCALPHA)
    dark.fill((10, 12, 18, DARKEN_ALPHA))
    surf.blit(dark, (0, 0))
    return surf


# ---------------------------------------------------------------- minimap
class MiniMap:
    """North-up satellite minimap in world east/north metres (home = 0, 0)."""

    def __init__(self, world):
        self.ok = False
        self.span_i = MAP_DEFAULT
        self.trail = deque(maxlen=TRAIL_MAX)
        self.last_trail = 0.0
        self.pins = []
        self.zones = []
        try:
            meta = json.load(open(os.path.join(REPO, "geo", "geo_meta.json")))
        except Exception as e:
            print(f"[map] no geo data ({e}); minimap disabled")
            return
        tiles = meta["tiles"]
        # tiles are relative to the home used when geo/ was fetched; shift them if
        # scenario.json home has moved since (rooftop pad, 24 Oct)
        try:
            h = json.load(open(os.path.join(REPO, "scenario.json")))["home"]
            gh = meta.get("home", h)
            m_lon = 111320.0 * math.cos(math.radians(h["lat"]))
            off_e, off_n = (gh["lon"] - h["lon"]) * m_lon, (gh["lat"] - h["lat"]) * 111320.0
        except Exception:
            off_e = off_n = 0.0
        tiles = [dict(t, east_min=t["east_min"] + off_e, north_min=t["north_min"] + off_n) for t in tiles]
        self.e0 = min(t["east_min"] for t in tiles)
        self.n1 = max(t["north_min"] + t["size_m"] for t in tiles)
        e1 = max(t["east_min"] + t["size_m"] for t in tiles)
        n0 = min(t["north_min"] for t in tiles)
        W, H = int(e1 - self.e0), int(self.n1 - n0)          # 1 m per pixel
        base = pygame.Surface((W, H))
        base.fill(MAP_RGB["bg"])
        for t in tiles:
            img = pygame.image.load(os.path.join(REPO, "geo", t["file"]))
            img = pygame.transform.smoothscale(img, (int(t["size_m"]), int(t["size_m"])))
            base.blit(img, (int(t["east_min"] - self.e0), int(self.n1 - (t["north_min"] + t["size_m"]))))
        # mip levels: metres per pixel 1, 2, 4, 8 (pick the one closest to the zoom)
        self.levels = {1: base}
        for r in (2, 4, 8):
            self.levels[r] = pygame.transform.smoothscale(base, (W // r, H // r))
        try:
            scenes = json.load(open(os.path.join(REPO, "worlds", f"{world}_scenes.json")))
            self.pins = sorted((cid, v["pin_en"][0], v["pin_en"][1]) for cid, v in scenes.items())
        except Exception as e:
            print(f"[map] no call pins ({e})")
        try:
            self.zones = json.load(open(os.path.join(REPO, "worlds", f"{world}_hazards.json"))).get("zones", [])
        except Exception as e:
            print(f"[map] no no-fly zones ({e})")
        self.font = None
        self.ok = True
        print(f"[map] {len(tiles)} imagery tiles, {len(self.pins)} call pins, {len(self.zones)} no-fly zones")

    def zoom(self, d):
        self.span_i = max(0, min(len(MAP_SPANS_M) - 1, self.span_i + d))

    def add_trail(self, e, n):
        now = time.time()
        if now - self.last_trail >= TRAIL_DT_S:
            self.trail.append((e, n))
            self.last_trail = now

    def draw(self, screen, pose):
        if not self.ok:
            return
        sw, sh = screen.get_size()
        side = int(sh * MAP_FRAC)
        x0, y0 = sw - side - 16, sh - side - 16
        span = MAP_SPANS_M[self.span_i]
        ce, cn, yaw = pose if pose else (0.0, 0.0, None)
        # source region in the closest mip level
        mpp_needed = span / side
        r = min(self.levels, key=lambda k: abs(math.log(k / max(mpp_needed, 0.25))))
        src = self.levels[r]
        px = (ce - self.e0) / r
        py = (self.n1 - cn) / r
        half = span / 2 / r
        crop = pygame.Surface((int(2 * half), int(2 * half)))
        crop.fill(MAP_RGB["bg"])
        crop.blit(src, (0, 0), pygame.Rect(int(px - half), int(py - half), int(2 * half), int(2 * half)))
        view = pygame.transform.smoothscale(crop, (side, side))
        k = side / span                                   # screen px per metre

        def to_px(e, n):
            return (int((e - ce) * k + side / 2), int((cn - n) * k + side / 2))

        if self.font is None or self.font_side != side:
            self.font = pygame.font.SysFont("dejavusans", max(13, side // 24), bold=True)
            self.font_side = side
        # no-fly zones: translucent dark red, darker rim, radar-style range ring
        if self.zones:
            lay = pygame.Surface((side, side), pygame.SRCALPHA)
            for z in self.zones:
                zx, zy = to_px(z["east_m"], z["north_m"])
                zr = max(3, int(z["radius_m"] * k))
                if zx + zr < 0 or zx - zr > side or zy + zr < 0 or zy - zr > side:
                    continue
                pygame.draw.circle(lay, MAP_RGB["nfz_fill"], (zx, zy), zr)
                pygame.draw.circle(lay, MAP_RGB["nfz_rim"] + (230,), (zx, zy), zr, max(2, side // 220))
                if zr > 10:
                    pygame.draw.circle(lay, MAP_RGB["nfz_rim"] + (140,), (zx, zy), int(zr * 0.6), 1)
                    pygame.draw.line(lay, MAP_RGB["nfz_rim"] + (140,), (zx - zr, zy), (zx + zr, zy), 1)
                    pygame.draw.line(lay, MAP_RGB["nfz_rim"] + (140,), (zx, zy - zr), (zx, zy + zr), 1)
            view.blit(lay, (0, 0))
        # trail
        if len(self.trail) > 1:
            pts = [to_px(e, n) for e, n in self.trail]
            pygame.draw.lines(view, MAP_RGB["trail"], False, pts, 2)
        def label(txt, x, y, col=(255, 255, 255)):
            t = self.font.render(txt, True, col)
            x = min(max(int(x), 2), side - t.get_width() - 8)
            y = min(max(int(y), 2), side - t.get_height() - 4)
            bg = pygame.Surface((t.get_width() + 6, t.get_height() + 2), pygame.SRCALPHA)
            bg.fill((0, 0, 0, 150))
            view.blit(bg, (x, y)); view.blit(t, (x + 3, y + 1))

        # home
        hx, hy = to_px(0.0, 0.0)
        hs = max(8, side // 36)
        pygame.draw.rect(view, MAP_RGB["home"], pygame.Rect(hx - hs, hy - hs, 2 * hs, 2 * hs), 2)
        t = self.font.render("H", True, MAP_RGB["home"])
        view.blit(t, (hx - t.get_width() // 2, hy - t.get_height() // 2))
        # call pins; calls outside the view get a marker on the edge pointing at them
        rr = max(7, side // 36)
        m = rr + 4
        for cid, e, n in self.pins:
            x, y = to_px(e, n)
            if m <= x <= side - m and m <= y <= side - m:
                pygame.draw.circle(view, MAP_RGB["pin"], (x, y), rr)
                pygame.draw.circle(view, (0, 0, 0), (x, y), rr, 2)
                pygame.draw.circle(view, (0, 0, 0), (x, y), max(2, rr // 3))
                label(cid, x + rr + 2, y - rr)
            else:
                cx_, cy_ = side / 2, side / 2
                ang = math.atan2(y - cy_, x - cx_)
                f = min((side / 2 - m) / max(abs(math.cos(ang)), 1e-6), (side / 2 - m) / max(abs(math.sin(ang)), 1e-6))
                ex, ey = cx_ + f * math.cos(ang), cy_ + f * math.sin(ang)
                tri = [(ex + rr * math.cos(ang), ey + rr * math.sin(ang)),
                       (ex + rr * math.cos(ang + 2.4), ey + rr * math.sin(ang + 2.4)),
                       (ex + rr * math.cos(ang - 2.4), ey + rr * math.sin(ang - 2.4))]
                pygame.draw.polygon(view, MAP_RGB["pin"], tri)
                pygame.draw.polygon(view, (0, 0, 0), tri, 1)
                label(cid, ex - 1.8 * rr * math.cos(ang) - 30, ey - 1.8 * rr * math.sin(ang) - 8)
        # drone (arrow along heading; yaw is ENU: 0 = east, counter-clockwise)
        if pose and yaw is None:                          # fallback feed has no heading
            pygame.draw.circle(view, MAP_RGB["drone"], to_px(ce, cn), max(6, side // 45))
            pygame.draw.circle(view, (0, 0, 0), to_px(ce, cn), max(6, side // 45), 2)
        elif pose:
            dx, dy = to_px(ce, cn)
            L = max(12, side // 14)
            ang = -yaw                                    # screen y points south
            tip = (dx + L * math.cos(ang), dy + L * math.sin(ang))
            left = (dx + 0.6 * L * math.cos(ang + 2.5), dy + 0.6 * L * math.sin(ang + 2.5))
            right = (dx + 0.6 * L * math.cos(ang - 2.5), dy + 0.6 * L * math.sin(ang - 2.5))
            pygame.draw.polygon(view, MAP_RGB["drone"], [tip, left, (dx, dy), right])
            pygame.draw.polygon(view, (0, 0, 0), [tip, left, (dx, dy), right], 2)
        # scale bar (round length ~ quarter of the span) and north arrow
        bar_m = [b for b in (25, 50, 100, 200, 250, 500, 1000) if b <= span / 3][-1]
        bx, by = 12, side - 14
        pygame.draw.line(view, (255, 255, 255), (bx, by), (bx + int(bar_m * k), by), 3)
        t = self.font.render(f"{bar_m} m", True, (255, 255, 255))
        view.blit(t, (bx, by - t.get_height() - 2))
        t = self.font.render("N", True, (255, 255, 255))          # north arrow, top-left
        nx = 10 + t.get_width() // 2
        pygame.draw.polygon(view, (255, 255, 255), [(nx, 8), (nx - 7, 22), (nx + 7, 22)])
        view.blit(t, (10, 24))
        screen.blit(view, (x0, y0))
        pygame.draw.rect(screen, MAP_RGB["frame"], pygame.Rect(x0 - 2, y0 - 2, side + 4, side + 4), 2)


# ---------------------------------------------------------------- status strip
def draw_status(screen, state, font):
    sw, _ = screen.get_size()
    h = font.get_height() + 16
    strip = pygame.Surface((sw, h), pygame.SRCALPHA)
    strip.fill((10, 12, 16, 190))
    screen.blit(strip, (0, 0))
    eg = state.get("egle_state") or "—"
    col = STATE_RGB.get(eg, (220, 220, 220))
    txt = STATE_TEXT.get(eg, eg)
    if eg == "AUTONOMOUS":
        intent = INTENT_LABEL.get(state.get("current_intent") or "", "")
        wp = state.get("current_wp")
        if intent:
            txt += f"  ·  {intent}" + (f" → {wp}" if wp and wp not in ("None", "BASE") else "")
    t = font.render(txt, True, col)
    pygame.draw.rect(screen, col, pygame.Rect(12, 8, 10, h - 16))
    screen.blit(t, (30, 8))
    parts = []
    if "battery_pct" in state:
        b = state["battery_pct"]
        bcol = (255, 90, 80) if b <= 5 else ((255, 200, 60) if state.get("battery_low") else (230, 230, 230))
        parts.append((f"Battery {b:.0f} %", bcol))
    c = state.get("condition_pct")
    if c is not None:
        if state.get("nofly_zone"):
            parts.append((f"Condition {c:.0f} %  ·  IN NO-FLY ZONE", (255, 90, 80)))
        else:
            parts.append((f"Condition {c:.0f} %", (255, 90, 80) if c <= 25 else ((255, 200, 60) if c <= 60 else (230, 230, 230))))
    if parts:
        rend = [font.render(tx, True, col) for tx, col in parts]
        gap = font.get_height() * 2
        x = sw // 2 - (sum(r.get_width() for r in rend) + gap * (len(rend) - 1)) // 2
        for r in rend:
            screen.blit(r, (x, 8)); x += r.get_width() + gap
    ft = state.get("flight_time")
    if ft:
        t = font.render(ft, True, (230, 230, 230))
        screen.blit(t, (sw - t.get_width() - 16, 8))


# ---------------------------------------------------------------- block end
def block_number():
    try:
        path = os.environ.get("UAV_SESSION", os.path.join(REPO, "session.json"))
        return json.load(open(path)).get("block", {}).get("block")
    except Exception:
        return None


def draw_block_end(screen, font, n):
    """Neutral end screen (Experiment Design rec. 5): no score, no performance data,
    nothing that identifies the condition."""
    screen.fill((12, 14, 18))
    sw, sh = screen.get_size()
    lines = [f"Block {n} complete" if n else "Block complete", "Please complete the questionnaire."]
    y = sh // 2 - font.get_height()
    for ln in lines:
        t = font.render(ln, True, (225, 225, 225))
        screen.blit(t, (sw // 2 - t.get_width() // 2, y))
        y += int(font.get_height() * 1.6)


# ---------------------------------------------------------------- main
def main():
    topic = sys.argv[sys.argv.index("--topic") + 1] if "--topic" in sys.argv else None
    node = Node()
    if topic is None:
        print("Looking for the egle_drone camera topic...")
        topic = find_topic(node)
        if topic is None:
            sys.exit("No camera topic found. Is Gazebo running with egle_drone?")
    world = topic.split("/")[2] if topic.startswith("/world/") else None
    latest = Latest()
    if not node.subscribe(Image, topic, latest.image_cb):
        sys.exit(f"Could not subscribe to {topic}")
    if world and not node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", latest.pose_cb):
        print("[map] could not subscribe to the drone pose; using the pilot's state file")
    print(f"Camera: {topic}")

    # Monitor placement (displays.json via tools/displays.py; the launcher sets
    # UAV_CAMERA_GEOM="x y w h"): borderless window covering that monitor.
    geom = os.environ.get("UAV_CAMERA_GEOM", "").split()
    if len(geom) == 4:
        os.environ["SDL_VIDEO_WINDOW_POS"] = f"{geom[0]},{geom[1]}"
    pygame.display.init()
    pygame.font.init()
    pygame.joystick.init()
    pads = {}
    if len(geom) == 4:
        screen = pygame.display.set_mode((int(geom[2]), int(geom[3])), pygame.NOFRAME)
    else:
        flags = pygame.FULLSCREEN if "--fullscreen" in sys.argv else pygame.RESIZABLE
        screen = pygame.display.set_mode((1280, 800), flags)
    pygame.display.set_caption(WINDOW_TITLE)
    mm = MiniMap(world) if world else None
    overlay, overlay_size, status_font, end_font, block_n = None, None, None, None, None
    state, state_t = {}, 0.0
    clock = pygame.time.Clock()
    jsd, js_on, js_retry, js_prev = None, False, 0.0, (0, 0, 0)
    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == getattr(pygame, "JOYDEVICEADDED", -1):
                j = pygame.joystick.Joystick(ev.device_index); j.init(); pads[j.get_instance_id()] = j
            elif ev.type == getattr(pygame, "JOYDEVICEREMOVED", -1):
                pads.pop(ev.instance_id, None)
            elif ev.type == pygame.JOYBUTTONDOWN and mm and not js_on:
                if ev.button == DPAD_UP:
                    mm.zoom(-1)
                elif ev.button == DPAD_DOWN:
                    mm.zoom(+1)
            elif ev.type == pygame.JOYHATMOTION and mm and ev.value[1] != 0 and not js_on:   # pads that report the D-pad as a hat
                mm.zoom(-1 if ev.value[1] > 0 else +1)
        now = time.time()
        # D-pad zoom read from /dev/input/js* so it works whatever window has focus
        if js_joystick is not None and os.environ.get("UAV_JOY_BACKEND", "js") == "js":
            if (jsd is None or not jsd.get_init()) and now >= js_retry:
                js_retry = now + 1.0
                jsd = js_joystick.open_first()
                js_prev = (0, 0, 0)
            js_on = jsd is not None and jsd.get_init()
            if js_on and mm:
                hy = jsd.get_hat(0)[1] if jsd.get_numhats() else 0
                cur = (jsd.get_button(DPAD_UP), jsd.get_button(DPAD_DOWN), hy)
                if (cur[0] and not js_prev[0]) or (cur[2] > 0 and js_prev[2] <= 0):
                    mm.zoom(-1)
                if (cur[1] and not js_prev[1]) or (cur[2] < 0 and js_prev[2] >= 0):
                    mm.zoom(+1)
                js_prev = cur
        if now - state_t > 0.25:
            state, state_t = read_state(), now
        with latest.lock:
            fr, pose = latest.frame, latest.pose
        if pose is None and state.get("position"):               # fallback: pilot NED -> east/north
            n_, e_, _ = state["position"]
            pose = (e_, n_, None)
        sw, sh = screen.get_size()
        if state.get("egle_state") == "LANDED":                # block over: neutral screen only
            if status_font is None or end_font is None:
                status_font = pygame.font.SysFont("dejavusans", max(16, sh // 32), bold=True)
                end_font = pygame.font.SysFont("dejavusans", max(24, sh // 18), bold=True)
            if block_n is None:
                block_n = block_number() or 0
            draw_block_end(screen, end_font, block_n or None)
            pygame.display.flip()
            clock.tick(10)
            continue
        if fr is not None and len(fr[2]) >= fr[0] * fr[1] * 3:
            w, h, data = fr
            img = pygame.image.frombuffer(data[: w * h * 3], (w, h), "RGB")
            screen.blit(pygame.transform.smoothscale(img, (sw, sh)), (0, 0))
            if overlay_size != (sw, sh):
                overlay, overlay_size = rain_overlay((sw, sh)), (sw, sh)
                status_font = pygame.font.SysFont("dejavusans", max(16, sh // 32), bold=True)
            screen.blit(overlay, (0, 0))
        else:
            screen.fill((20, 22, 26))
            if status_font is None:
                status_font = pygame.font.SysFont("dejavusans", max(16, sh // 32), bold=True)
        if mm:
            if pose:
                mm.add_trail(pose[0], pose[1])
            mm.draw(screen, pose)
        draw_status(screen, state, status_font)
        pygame.display.flip()
        clock.tick(30)
    pygame.quit()


if __name__ == "__main__":
    main()
