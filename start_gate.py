"""
Start gate: waits until the pilot presses START on the controller (no keyboard
start: Ben, 23 Oct), then exits; the launcher then creates /tmp/uav_start_signal,
which the already-running pilot script is waiting for.

Doesn't need window focus for the
controller button.

Usage:
    python3 start_gate.py            # wait for START on the controller
    python3 start_gate.py --detect   # print the number of every button pressed
                                     # (use this to confirm which one is Start)
"""
import os
import sys
import time

# Must be set before pygame is imported
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")                  # no window needed
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1") # button works without focus
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame

# Xbox 360 on Linux usually reports Start as button 7 (A=0 B=1 X=2 Y=3 LB=4 RB=5 Back=6).
# If Start doesn't work, run with --detect and put the number it prints here.
START_BUTTON = int(os.environ.get("UAV_START_BUTTON", "7"))

detect = "--detect" in sys.argv

pygame.display.init()
pygame.joystick.init()
pads = []


def scan():
    """(Re)attach controllers; called at start and every RESCAN_S while none is attached."""
    global pads
    pygame.joystick.quit()
    pygame.joystick.init()
    pads = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    for p in pads:
        p.init()
    return pads


RESCAN_S = 1.0
scan()
if pads:
    print(f"Controller found: {pads[0].get_name()}")
else:
    print("No controller found yet — plug it in (checked every second).")

if detect:
    print("Press controller buttons to see their numbers. Ctrl+C to exit.")
else:
    print(">>> Press START on the controller to START the mission "
          "(drone arms and takes off ~6 s later; control is live once airborne)")
sys.stdout.flush()

next_scan = time.time() + RESCAN_S
try:
    while True:
        if not pads and time.time() >= next_scan:
            next_scan = time.time() + RESCAN_S
            if scan():
                print(f"Controller connected: {pads[0].get_name()}")
                sys.stdout.flush()
        for ev in pygame.event.get():
            if ev.type == getattr(pygame, "JOYDEVICEREMOVED", -1):
                print("Controller disconnected — waiting for it.")
                sys.stdout.flush()
                pads = []
            elif ev.type == pygame.JOYBUTTONDOWN:
                if detect:
                    print(f"  button {ev.button}")
                    sys.stdout.flush()
                elif ev.button == START_BUTTON:
                    print("START pressed on controller.")
                    sys.exit(0)
        time.sleep(0.02)
except KeyboardInterrupt:
    print("\nStart cancelled.")
    sys.exit(1)
