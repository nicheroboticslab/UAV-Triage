#!/usr/bin/env python3
"""
EGLe camera window (E3, C2, E7).

Shows the drone's 45-degree forward camera from Gazebo in its own window
(for the projector). Reads the image directly from Gazebo transport
(gz-transport13 Python bindings; no ROS bridge, no OpenCV).

C2 look: static darkening plus a fixed, low-contrast rain-streak overlay.
E7: nothing flashes or strobes; the overlay does not change over time.

    python3 camera_view.py                 # finds the camera topic automatically
    python3 camera_view.py --topic /world/.../image
    python3 camera_view.py --fullscreen    # for the projector
"""
import os
import random
import sys
import threading
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.transport13 import Node  # noqa: E402

DARKEN_ALPHA = 70        # 0-255, overall dimming for overcast/rain
RAIN_STREAKS = 900
RAIN_ALPHA = 38          # low contrast
WINDOW_TITLE = "EGLe Camera"


class Latest:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None   # (w, h, bytes)
        self.t = 0.0

    def cb(self, msg):
        if msg.pixel_format_type not in (3, 0):   # RGB_INT8 = 3 in gz-msgs10; 0 = unknown -> try anyway
            pass
        with self.lock:
            self.frame = (msg.width, msg.height, bytes(msg.data))
            self.t = time.time()


def find_topic(node, timeout=30):
    end = time.time() + timeout
    while time.time() < end:
        for t in node.topic_list():
            if "egle_drone" in t and t.endswith("/image"):
                return t
        time.sleep(0.5)
    return None


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


def main():
    topic = sys.argv[sys.argv.index("--topic") + 1] if "--topic" in sys.argv else None
    node = Node()
    if topic is None:
        print("Looking for the egle_drone camera topic...")
        topic = find_topic(node)
        if topic is None:
            sys.exit("No camera topic found. Is Gazebo running with egle_drone?")
    latest = Latest()
    if not node.subscribe(Image, topic, latest.cb):
        sys.exit(f"Could not subscribe to {topic}")
    print(f"Camera: {topic}")

    pygame.display.init()
    flags = pygame.FULLSCREEN if "--fullscreen" in sys.argv else pygame.RESIZABLE
    screen = pygame.display.set_mode((960, 720), flags)
    pygame.display.set_caption(WINDOW_TITLE)
    overlay, overlay_size = None, None
    clock = pygame.time.Clock()
    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                running = False
        with latest.lock:
            fr = latest.frame
        if fr is not None:
            w, h, data = fr
            if len(data) >= w * h * 3:
                img = pygame.image.frombuffer(data[: w * h * 3], (w, h), "RGB")
                sw, sh = screen.get_size()
                img = pygame.transform.smoothscale(img, (sw, sh))
                if overlay_size != (sw, sh):
                    overlay, overlay_size = rain_overlay((sw, sh)), (sw, sh)
                screen.blit(img, (0, 0))
                screen.blit(overlay, (0, 0))
        else:
            screen.fill((20, 22, 26))
        pygame.display.flip()
        clock.tick(30)
    pygame.quit()


if __name__ == "__main__":
    main()
