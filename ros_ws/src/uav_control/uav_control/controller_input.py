#!/usr/bin/env python3
"""
EGLe Controller Input Thread
Replaces keyboard input with ShanWan Readgear gamepad
Drop-in replacement for the pynput keyboard listener in autonomous_uav_stage5.py
"""

import pygame
import threading
import time

# Deadzone for analog sticks — ignore small movements
DEADZONE = 0.15
# Step size per update cycle (matches keyboard step)
STEP = 0.5

class ControllerInput:
    def __init__(self, node):
        """
        node: reference to your UAVControlNode instance
        Provides same interface as keyboard listener
        """
        self.node = node
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self.running = False

    def _apply_deadzone(self, val):
        return val if abs(val) > DEADZONE else 0.0

    def _loop(self):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("[CONTROLLER] No joystick found — falling back to keyboard")
            return

        j = pygame.joystick.Joystick(0)
        j.init()
        print(f"[CONTROLLER] Connected: {j.get_name()}")
        print("[CONTROLLER] Left stick = move XY | Right stick = yaw/altitude")
        print("[CONTROLLER] X=Battery spoof | O=EKF spoof | △=Reset | L1=Force LLM")

        prev_buttons = [0] * j.get_numbuttons()

        while self.running:
            pygame.event.pump()

            # ── Analog sticks → target position ──────────────────
            lx = self._apply_deadzone(j.get_axis(0))   # left stick X → Y axis (strafe)
            ly = self._apply_deadzone(j.get_axis(1))   # left stick Y → X axis (forward/back)
            rx = self._apply_deadzone(j.get_axis(2))   # right stick X → yaw (ignored for now)
            ry = self._apply_deadzone(j.get_axis(3))   # right stick Y → altitude

            moved = False
            if ly != 0.0:
                self.node.target_pos[0] -= ly * STEP   # forward/back (NED: -ly = forward)
                moved = True
            if lx != 0.0:
                self.node.target_pos[1] += lx * STEP   # strafe left/right
                moved = True
            if ry != 0.0:
                self.node.target_pos[2] += ry * STEP   # altitude (NED: positive = down)
                moved = True

            if moved:
                self.node.last_input_time = time.time()
                if self.node.autonomous_mode:
                    self.node.autonomous_mode = False
                    print("[CONTROLLER] Manual override — autonomous mode OFF")

            # ── Buttons (edge-triggered — fires once on press) ────
            buttons = [j.get_button(i) for i in range(j.get_numbuttons())]

            def pressed(idx):
                return buttons[idx] == 1 and prev_buttons[idx] == 0

            # X button (0) → Battery spoof Y
            if pressed(0):
                print("[CONTROLLER] X pressed → Battery spoof (Y key)")
                self.node.battery_override = True
                self.node.battery_override_value = 10.0

            # O button (1) → EKF spoof T
            if pressed(1):
                print("[CONTROLLER] O pressed → EKF spoof (T key)")
                self.node.ekf_override = True
                self.node.ekf_override_value = 0.2

            # △ button (3) → Reset R
            if pressed(3):
                print("[CONTROLLER] △ pressed → Reset overrides (R key)")
                self.node.battery_override = False
                self.node.ekf_override = False

            # L1 button (4) → Force LLM re-query F
            if pressed(4):
                print("[CONTROLLER] L1 pressed → Force LLM re-query (F key)")
                self.node.force_llm_requery = True

            # Button 6 → Land (Space)
            if pressed(6):
                print("[CONTROLLER] Button 6 → Land")
                self.node.target_pos[2] = 0.0

            # Button 7 → Quit
            if pressed(7):
                print("[CONTROLLER] Button 7 → Quit")
                self.node.running = False

            prev_buttons = buttons[:]
            time.sleep(0.05)   # 20 Hz polling

        pygame.quit()
