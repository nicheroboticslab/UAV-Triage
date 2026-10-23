#!/usr/bin/env python3
"""
Generate the flood-water textures used by make_world.py (C5/C6, decisions 7.8).

    python3 tools/make_water_textures.py

Writes models/flood_water/textures/standing.png and flowing.png (1024 x 1024).
Deterministic (fixed seeds), so the files are identical on every run; the PNGs
are committed and this script documents how they were made.

standing: muddy brown with low-contrast blotches, no direction.
flowing:  same base, plus streaks along +u (the flow direction, = the water
          box's +x axis) and foam bands. Static: nothing moves, so nothing
          flickers (photosensitivity, decisions 7.8).
"""
import os
import random

from PIL import Image, ImageDraw, ImageFilter

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
OUT = os.path.join(REPO, "models", "flood_water", "textures")
N = 1024
BASE = (102, 112, 115)        # blue-grey-brown floodwater (Ben, 23 Oct: 'a bit more blue')


def blotches(img, seed, n, rmin, rmax, delta):
    rnd = random.Random(seed)
    layer = Image.new("RGB", img.size, BASE)
    d = ImageDraw.Draw(layer)
    for _ in range(n):
        x, y, r = rnd.randrange(N), rnd.randrange(N), rnd.randint(rmin, rmax)
        k = rnd.uniform(-delta, delta)
        col = tuple(max(0, min(255, int(c * (1 + k)))) for c in BASE)
        d.ellipse([x - r, y - r, x + r, y + r], fill=col)
    return layer.filter(ImageFilter.GaussianBlur(24))


def standing():
    img = blotches(Image.new("RGB", (N, N)), 11, 220, 30, 120, 0.10)
    d = ImageDraw.Draw(img, "RGBA")
    rnd = random.Random(12)
    for _ in range(40):                      # faint sky reflections, no direction
        x, y = rnd.randrange(N), rnd.randrange(N)
        r = rnd.randint(20, 70)
        d.ellipse([x - r, y - r // 3, x + r, y + r // 3], fill=(150, 150, 150, 18))
    return img.filter(ImageFilter.GaussianBlur(3))


def flowing():
    img = blotches(Image.new("RGB", (N, N)), 21, 220, 30, 120, 0.12)
    img = Image.blend(img, Image.new("RGB", (N, N), (128, 136, 134)), 0.25)   # more turbid
    d = ImageDraw.Draw(img, "RGBA")
    rnd = random.Random(22)
    for _ in range(1400):                    # streaks along +u (strong: flowing must read at a glance)
        x, y = rnd.randrange(-200, N), rnd.randrange(N)
        ln, w = rnd.randint(80, 320), rnd.choice((2, 2, 3, 4))
        a = rnd.randint(60, 140)
        d.line([x, y, x + ln, y + rnd.randint(-4, 4)], fill=(210, 200, 185, a), width=w)
    for _ in range(140):                     # foam bands
        x, y = rnd.randrange(N), rnd.randrange(N)
        for k in range(rnd.randint(4, 10)):
            dx, dy = x + k * rnd.randint(8, 18), y + rnd.randint(-6, 6)
            r = rnd.randint(3, 9)
            d.ellipse([dx - 2 * r, dy - r, dx + 2 * r, dy + r], fill=(235, 230, 220, rnd.randint(60, 130)))
    return img.filter(ImageFilter.GaussianBlur(1.2))


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    standing().save(os.path.join(OUT, "standing.png"), optimize=True)
    flowing().save(os.path.join(OUT, "flowing.png"), optimize=True)
    print("Wrote", OUT)
