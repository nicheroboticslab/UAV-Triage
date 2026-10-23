"""
Start gate: waits until the pilot presses START on the controller, or Enter
in this terminal, then exits; the launcher then creates /tmp/uav_start_signal,
which the already-running pilot script is waiting for.

Works without a controller (Enter only). Doesn't need window focus for the
controller button.

Usage:
    python3 start_gate.py            # wait for START or Enter
    python3 start_gate.py --detect   # print the number of every button pressed
                                     # (use this to confirm which one is Start)
"""
import os
import select
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
pads = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
for p in pads:
    p.init()

if pads:
    print(f"Controller found: {pads[0].get_name()}")
else:
    print("No controller found — keyboard only.")

if detect:
    print("Press controller buttons to see their numbers. Ctrl+C to exit.")
else:
    who = "START on the controller, or Enter here" if pads else "Enter here"
    print(f">>> Press {who} to START the mission (drone arms and takes off ~6 s later; control is live once airborne)")
sys.stdout.flush()

try:
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.JOYBUTTONDOWN:
                if detect:
                    print(f"  button {ev.button}")
                    sys.stdout.flush()
                elif ev.button == START_BUTTON:
                    print("START pressed on controller.")
                    sys.exit(0)
        if not detect:
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                sys.stdin.readline()
                print("START pressed (keyboard).")
                sys.exit(0)
        time.sleep(0.02)
except KeyboardInterrupt:
    print("\nStart cancelled.")
    sys.exit(1)
