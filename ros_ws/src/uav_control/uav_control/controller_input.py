"""
Xbox 360 controller input for the UAV-Triage pilot script.

Every controller action is converted into the SAME key event the keyboard would
send and passed to the pilot script's own on_key_press(). So the controller
behaves exactly like the verified keyboard controls: taking manual control,
the idle hand-back timer, logging, dataset records, and fault injection.

Layout (standard "Mode 2" flight controls, relative to where the drone faces):
  Left stick up/down      climb / descend (throttle)
  Left stick left/right   turn left / right (yaw)
  Right stick up/down     fly forward / back (pitch)
  Right stick left/right  fly sideways left / right (roll)
  A / B / X / Y        unassigned (reserved; any press counts as operator input)
  LB + RB together     emergency: return home and land (ends the block)
  Back                 progress stats (H)
  Start                not used in flight (it's the mission START button)

Operator input (A1): any stick outside the deadzone or any button press
resets the inaction timer, except LB+RB, which starts the emergency return
and must not be undone by its own press.
Fault injection (T/Y/Z/R) stays keyboard-only for the researcher.

Sticks glide the target smoothly (proportional, with a short leash ahead of the drone).
Works without window focus.

Check your controller's numbering:
    python3 controller_input.py --detect
"""
import math
import os
import sys
import threading
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame
from pynput import keyboard

# ---- Xbox 360 on Linux (xpad driver) ----
AX_LEFT_X, AX_LEFT_Y = 0, 1
AX_RIGHT_X, AX_RIGHT_Y = 3, 4
BTN_A, BTN_B, BTN_X, BTN_Y = 0, 1, 2, 3
BTN_LB, BTN_RB, BTN_BACK, BTN_START = 4, 5, 6, 7

DEADZONE = 0.20
POLL_HZ = 20
EXPO = 2.0            # stick curve: >1 = finer control near centre (1 = linear)

# Smooth mode: the target glides every cycle and stays on a short leash ahead of the drone
MAX_SPEED = 23.0      # m/s horizontal at full stick (D1, = MPC_XY_VEL_MAX)
MAX_CLIMB = 6.0       # m/s ascent at full stick (D1, = MPC_Z_VEL_MAX_UP)
MAX_DESCENT = 5.0     # m/s descent at full stick (D1, = MPC_Z_VEL_MAX_DN)
LEASH_XY = 8.0        # max metres the target may lead the drone horizontally
LEASH_Z = 3.0         # max metres the target may lead the drone vertically
MAX_YAW_RATE = 60.0   # deg/s turn rate at full stick
IDLE_RESET_S = 0.5    # sticks centred this long = next movement starts a new "burst"

# Fallback only (if the pilot script lacks target_pos/current_pos/last_key_time)
MAX_MOVE_RATE = 8.0
MAX_ALT_RATE = 3.0

# The pilot script's key handler: up = +north, down = -north,
# left = +east, right = -east, 'w' = climb, 's' = descend.
K_NORTH, K_SOUTH = keyboard.Key.up, keyboard.Key.down
K_EAST, K_WEST = keyboard.Key.left, keyboard.Key.right
K_CLIMB, K_DESCEND = keyboard.KeyCode.from_char('w'), keyboard.KeyCode.from_char('s')

# A/B/X/Y previously forced agent intents (1, 6, 3) and an LLM re-query (F),
# an operator-initiated path to autonomy that is not part of the study design.
# They are unassigned for now and may be reassigned later.
BUTTON_KEYS = {
    BTN_BACK: keyboard.KeyCode.from_char('h'),
}
NO_INPUT_RESET = {BTN_LB, BTN_RB}
EMERGENCY_LAND = keyboard.KeyCode.from_char('7')


class ControllerInput:
    def __init__(self, node, joystick=None):
        self.node = node
        self.running = True
        self._joystick = joystick          # injectable for testing
        self._acc = {}                     # per-direction step accumulators (fallback)
        self._burst = False
        self._idle = 0.0
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self.running = False

    def _log(self, msg):
        try:
            self.node.get_logger().info(msg)
        except Exception:
            print(msg)

    def _send(self, key):
        try:
            self.node.on_key_press(key)
        except Exception as e:
            self._log(f"[CONTROLLER] key handler error: {e!r}")

    def _axis(self, j, idx):
        v = j.get_axis(idx) if idx < j.get_numaxes() else 0.0
        return 0.0 if abs(v) < DEADZONE else v

    def _smooth_ok(self):
        n = self.node
        return all(hasattr(n, a) for a in ("target_pos", "current_pos", "last_key_time"))

    @staticmethod
    def _curve(v):
        return (abs(v) ** EXPO) * (1 if v > 0 else -1) if v else 0.0

    def _commanded_yaw(self):
        """The heading the pilot script is currently sending to PX4 (see its sp.yaw line)."""
        n = self.node
        if getattr(n, "manual_yaw", None) is not None and self._in_manual():
            return n.manual_yaw
        if getattr(n, "loitering", False) and hasattr(n, "circle_angle"):
            return n.circle_angle + math.pi / 2.0
        return 0.0

    def _in_manual(self):
        return getattr(getattr(self.node, "drone_state", None), "name", "") == "MANUAL"

    def _smooth(self, fwd, right, up, turn, dt):
        n = self.node
        active = bool(fwd or right or up or turn)
        now = time.time()
        if not active:
            # Brake-then-hold: after the sticks are released, keep the target on
            # the vehicle until it has slowed, so PX4 brakes to a stop instead of
            # overshooting the last target and flying back to it.
            if self._burst and self._in_manual():
                cur = list(n.current_pos)
                prev = getattr(self, "_prev_pos", None)
                speed = (math.hypot(cur[0] - prev[0], cur[1] - prev[1]) / dt) if prev else 99.0
                self._prev_pos = cur
                if speed > 0.5:
                    n.target_pos[0], n.target_pos[1], n.target_pos[2] = cur[0], cur[1], n.target_pos[2]
                    self._idle = 0.0
            self._idle += dt
            if self._burst and self._idle > IDLE_RESET_S:
                self._burst = False
            if not self._burst and not self._in_manual() and hasattr(n, "manual_yaw"):
                n.manual_yaw = None               # AI flying: it controls heading again
            n.manual_vel = None                   # sticks centred: no velocity feed-forward
            return
        self._idle = 0.0
        self._prev_pos = None
        if not self._burst:
            start_yaw = self._commanded_yaw()     # keep the heading the drone already has
            # One real key press through the pilot script so MANUAL takeover and
            # logging happen exactly as with the keyboard.
            dom = max((abs(fwd), K_NORTH if fwd > 0 else K_SOUTH),
                      (abs(right), K_EAST if right > 0 else K_WEST),
                      (abs(up), K_CLIMB if up > 0 else K_DESCEND),
                      (abs(turn), K_NORTH), key=lambda t: t[0])[1]
            self._send(dom)
            if hasattr(n, "loitering"):
                n.loitering = False               # stop the hold-circle rewriting the target
            n.manual_yaw = start_yaw
            cur = list(n.current_pos)
            n.target_pos[0], n.target_pos[1], n.target_pos[2] = cur[0], cur[1], cur[2]
            self._burst = True
        n.last_key_time = now                      # keeps the AI from taking over

        # Turn
        yaw = n.manual_yaw + self._curve(turn) * math.radians(MAX_YAW_RATE) * dt
        n.manual_yaw = math.atan2(math.sin(yaw), math.cos(yaw))   # wrap to [-pi, pi]

        # Body-frame stick -> world north/east (PX4 yaw: 0 = north, + = clockwise)
        f = self._curve(fwd) * MAX_SPEED * dt
        r = self._curve(right) * MAX_SPEED * dt
        cu = self._curve(up)
        vz = -cu * (MAX_CLIMB if cu > 0 else MAX_DESCENT)   # NED: up = negative
        c, sn = math.cos(n.manual_yaw), math.sin(n.manual_yaw)
        cur = n.current_pos
        t = n.target_pos
        t[0] += c * f - sn * r
        t[1] += sn * f + c * r
        t[2] += vz * dt
        # Velocity feed-forward so full stick reaches the PX4 cap; the leash
        # below only bounds how far the position target can lead.
        n.manual_vel = [(c * f - sn * r) / dt, (sn * f + c * r) / dt, vz]

        # Leash: never let the target run far ahead of the drone
        dx, dy = t[0] - cur[0], t[1] - cur[1]
        d = (dx * dx + dy * dy) ** 0.5
        if d > LEASH_XY:
            t[0] = cur[0] + dx * LEASH_XY / d
            t[1] = cur[1] + dy * LEASH_XY / d
        dz = t[2] - cur[2]
        if abs(dz) > LEASH_Z:
            t[2] = cur[2] + (LEASH_Z if dz > 0 else -LEASH_Z)
        if t[2] > -1.0:                            # never command below 1 m altitude
            t[2] = -1.0

    def _drive(self, name, value, key_pos, key_neg, max_rate, dt):
        """Proportional stepping: accumulate deflection, emit one key per whole step."""
        acc = self._acc.get(name, 0.0)
        if value == 0.0 or (acc != 0.0 and (acc > 0) != (value > 0)):
            acc = 0.0
        acc += value * max_rate * dt
        while acc >= 1.0:
            self._send(key_pos); acc -= 1.0
        while acc <= -1.0:
            self._send(key_neg); acc += 1.0
        self._acc[name] = acc

    def _open(self):
        if self._joystick is not None:
            return self._joystick
        pygame.display.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self._log("[CONTROLLER] No controller found — keyboard only")
            return None
        j = pygame.joystick.Joystick(0)
        j.init()
        self._log(f"[CONTROLLER] Connected: {j.get_name()} "
                  f"({j.get_numaxes()} axes, {j.get_numbuttons()} buttons)")
        return j

    def step(self, j, prev, dt):
        """One polling cycle. Returns the current button states."""
        if getattr(self.node, "block_ended", False):
            # Block over (emergency land): ignore all controller input.
            return [j.get_button(i) for i in range(j.get_numbuttons())]
        # Sticks. Pygame Y axes are negative when pushed UP.
        fwd = -self._axis(j, AX_RIGHT_Y)      # pitch
        right = self._axis(j, AX_RIGHT_X)     # roll
        up = -self._axis(j, AX_LEFT_Y)        # throttle
        turn = self._axis(j, AX_LEFT_X)       # yaw (+ = clockwise)
        if self._smooth_ok():
            self._smooth(fwd, right, up, turn, dt)
        else:
            self._drive("ns", fwd, K_NORTH, K_SOUTH, MAX_MOVE_RATE, dt)
            self._drive("ew", right, K_EAST, K_WEST, MAX_MOVE_RATE, dt)
            self._drive("alt", up, K_CLIMB, K_DESCEND, MAX_ALT_RATE, dt)

        buttons = [j.get_button(i) for i in range(j.get_numbuttons())]
        def pressed(i):
            return i < len(buttons) and buttons[i] and not prev[i]

        both_bumpers = (buttons[BTN_LB] and buttons[BTN_RB]) if len(buttons) > BTN_RB else False
        was_both = (prev[BTN_LB] and prev[BTN_RB]) if len(prev) > BTN_RB else False
        if both_bumpers and not was_both:
            self._log("[CONTROLLER] LB+RB → emergency return home and land")
            if hasattr(self.node, "emergency_return"):
                self.node.emergency_return()
            else:
                self._send(EMERGENCY_LAND)
        for btn, key in BUTTON_KEYS.items():
            if pressed(btn):
                self._send(key)
        # A1: any new button press (except the emergency-land bumpers) counts as
        # operator input. If the agent was flying, the pilot script hands back.
        if any(pressed(i) for i in range(len(buttons)) if i not in NO_INPUT_RESET):
            if hasattr(self.node, "last_key_time"):
                self.node.last_key_time = time.time()
        return buttons

    def _loop(self):
        j = self._open()
        if j is None:
            return
        prev = [0] * j.get_numbuttons()
        dt = 1.0 / POLL_HZ
        while self.running:
            if self._joystick is None:
                pygame.event.pump()
            prev = self.step(j, prev, dt)
            time.sleep(dt)


def _detect():
    pygame.display.init(); pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit("No controller found.")
    j = pygame.joystick.Joystick(0); j.init()
    print(f"{j.get_name()}: {j.get_numaxes()} axes, {j.get_numbuttons()} buttons. Ctrl+C to exit.")
    print("Move ONE stick at a time; note which axis number changes.")
    last = None
    try:
        while True:
            pygame.event.pump()
            axes = tuple(round(j.get_axis(i), 1) for i in range(j.get_numaxes()))
            btns = [i for i in range(j.get_numbuttons()) if j.get_button(i)]
            state = (axes, tuple(btns))
            if state != last:
                print("axes " + " ".join(f"{i}:{v:+.1f}" for i, v in enumerate(axes)), "| buttons", btns)
                last = state
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    if "--detect" in sys.argv:
        _detect()
    else:
        print(__doc__)
