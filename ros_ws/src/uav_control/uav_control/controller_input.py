"""
Xbox 360 controller input for the UAV-Triage pilot script.

Layout (23 Oct remap; body-relative, i.e. relative to where the drone faces):
  Left stick up/down      fly forward / back (pitch)
  Left stick left/right   turn left / right (yaw)
  LT / RT (analog)        slide left / right (roll)
  RB held                 climb   (ALT_RATE m/s, ramped)
  LB held                 descend (ALT_RATE m/s, ramped)
  LB + RB held 0.5 s      emergency: return home and land (ends the block)
  Right stick up/down     camera pitch (0 = horizon .. 90 deg down)
  Right stick left/right  camera yaw relative to the nose (+-90 deg)
  R3 (right stick click)  recentre camera (45 deg down, forward)
  Start                   mission START (read by start_gate.py, not here)
  Back, A/B/X/Y, D-pad, L3  unassigned (any press still counts as operator input)

Operator input (A1): any stick/trigger outside the deadzone or any button
press resets the inaction timer, except LB+RB together (emergency), which
must not be undone by its own press.

Handling: sticks set a desired velocity; the commanded velocity moves toward
it at a limited acceleration (ACCEL_XY / BRAKE_XY / ACCEL_Z), and the
position target is advanced along it. Releasing the sticks therefore brakes
along a smooth profile instead of snapping, which removes overshoot and
bounce-back. Stick curve: deadzone removed and rescaled, then RC-style expo.

Every poll writes nothing; every POLL_HZ/LOG_HZ polls one line goes to
~/uav_dataset/controls_<session>.jsonl (D6/H data, used for handling checks).

Check your controller's numbering:
    python3 controller_input.py --detect
"""
import json
import math
import os
import sys
import threading
import time
from datetime import datetime

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame
from pynput import keyboard

# ---- Xbox 360 on Linux (xpad driver); confirm with --detect ----
AX_LEFT_X, AX_LEFT_Y = 0, 1
AX_LT = 2
AX_RIGHT_X, AX_RIGHT_Y = 3, 4
AX_RT = 5
BTN_A, BTN_B, BTN_X, BTN_Y = 0, 1, 2, 3
BTN_LB, BTN_RB, BTN_BACK, BTN_START = 4, 5, 6, 7
BTN_L3, BTN_R3 = 9, 10

POLL_HZ = 20
LOG_HZ = 10
DEADZONE = 0.15       # sticks: |v| below this = 0; output rescaled to start at 0 at the edge
TRIGGER_DEADZONE = 0.05
EXPO = 0.6            # 0 = linear, 1 = cubic; RC-style blend out = (1-e)*x + e*x^3

# Vehicle (D1: M30T caps = PX4 MPC_* params)
MAX_SPEED = 23.0      # m/s horizontal at full stick (MPC_XY_VEL_MAX)
ALT_RATE = 3.0        # m/s climb (RB) / descent (LB)
ACCEL_XY = 5.0        # m/s^2 speeding up
BRAKE_XY = 8.0        # m/s^2 slowing down (stick released or reduced)
ACCEL_Z = 6.0         # m/s^2 vertical ramp (0 -> 3 m/s in 0.5 s)
MAX_YAW_RATE = 60.0   # deg/s at full stick
LEASH_XY = 8.0        # max metres the target may lead the drone horizontally
LEASH_Z = 3.0
MIN_ALT_M = 1.0       # never command below 1 m
IDLE_RESET_S = 0.5    # stopped and centred this long = next input starts a new burst

EMERGENCY_HOLD_S = 0.5

# Camera gimbal (degrees). Pitch: 0 = horizon, 90 = straight down.
GIMBAL_PITCH_RANGE = (0.0, 90.0)
GIMBAL_YAW_RANGE = (-90.0, 90.0)
GIMBAL_HOME = (45.0, 0.0)
GIMBAL_RATE = 60.0    # deg/s at full stick

# The pilot script's key handler: up = +north, down = -north, left = +east,
# right = -east, 'w' = climb, 's' = descend. One key event per burst goes
# through it so MANUAL takeover and logging happen exactly as with the keyboard.
K_NORTH, K_SOUTH = keyboard.Key.up, keyboard.Key.down
K_EAST, K_WEST = keyboard.Key.left, keyboard.Key.right
K_CLIMB, K_DESCEND = keyboard.KeyCode.from_char('w'), keyboard.KeyCode.from_char('s')
EMERGENCY_LAND = keyboard.KeyCode.from_char('7')

NO_INPUT_RESET = {BTN_LB, BTN_RB}   # bumpers count via the altitude path instead
DATASET_DIR = os.path.expanduser("~/uav_dataset")


def shape(v, deadzone=DEADZONE):
    """Deadzone removed and rescaled to 0..1, then expo."""
    a = abs(v)
    if a < deadzone:
        return 0.0
    x = min(1.0, (a - deadzone) / (1.0 - deadzone))
    y = (1.0 - EXPO) * x + EXPO * x ** 3
    return y if v > 0 else -y


def approach(cur, target, up_rate, down_rate, dt):
    """Move cur toward target; down_rate applies when |target| < |cur| (braking)."""
    rate = down_rate if abs(target) < abs(cur) or cur * target < 0 else up_rate
    step = rate * dt
    if abs(target - cur) <= step:
        return target
    return cur + step if target > cur else cur - step


class ControllerInput:
    def __init__(self, node, joystick=None):
        self.node = node
        self.running = True
        self._joystick = joystick          # injectable for testing
        self._burst = False
        self._idle = 0.0
        self._v = [0.0, 0.0]               # commanded world velocity north, east (m/s)
        self._vz = 0.0                     # commanded climb rate (m/s, + = up)
        self._both_since = None
        self._emergency_sent = False
        self._trig_seen_rest = {AX_LT: False, AX_RT: False}
        self._log_every = max(1, POLL_HZ // LOG_HZ)
        self._tick = 0
        self._log_path = None
        self._last = {}
        if not hasattr(node, "gimbal_cmd"):
            node.gimbal_cmd = list(GIMBAL_HOME)
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

    # ---------- raw input ----------
    def _axis(self, j, idx):
        return j.get_axis(idx) if idx < j.get_numaxes() else 0.0

    def _trigger(self, j, idx):
        """0..1. Triggers rest at -1 on xpad; some drivers report 0 until first
        moved, so a trigger is ignored until it has been seen at rest."""
        raw = self._axis(j, idx)
        if raw <= -0.9:
            self._trig_seen_rest[idx] = True
        if not self._trig_seen_rest.get(idx, False):
            return 0.0
        v = (raw + 1.0) / 2.0
        return 0.0 if v < TRIGGER_DEADZONE else v

    def _in_manual(self):
        return getattr(getattr(self.node, "drone_state", None), "name", "") == "MANUAL"

    def _commanded_yaw(self):
        """Heading the pilot script last sent to PX4, so takeover keeps it."""
        n = self.node
        if getattr(n, "manual_yaw", None) is not None and self._in_manual():
            return n.manual_yaw
        return float(getattr(n, "last_sp_yaw", 0.0) or 0.0)

    # ---------- flight ----------
    def _fly(self, fwd, right, up, turn, dt):
        n = self.node
        active = bool(fwd or right or up or turn)
        now = time.time()

        if active and not self._burst:
            dom = max((abs(fwd), K_NORTH if fwd > 0 else K_SOUTH),
                      (abs(right), K_EAST if right > 0 else K_WEST),
                      (abs(up), K_CLIMB if up > 0 else K_DESCEND),
                      (abs(turn), K_NORTH), key=lambda t: t[0])[1]
            start_yaw = self._commanded_yaw()
            self._send(dom)                        # MANUAL takeover + logging via the pilot
            if hasattr(n, "loitering"):
                n.loitering = False
            n.manual_yaw = start_yaw
            cur = list(n.current_pos)
            n.target_pos[0], n.target_pos[1], n.target_pos[2] = cur[0], cur[1], cur[2]
            self._v = [0.0, 0.0]
            self._vz = 0.0
            self._burst = True

        if not self._burst:
            if not self._in_manual() and hasattr(n, "manual_yaw"):
                n.manual_yaw = None                # agent flying: it controls heading
            n.manual_vel = None
            return

        if active:
            n.last_key_time = now                  # keeps the agent from taking over
            self._idle = 0.0

        # Yaw
        if turn:
            yaw = n.manual_yaw + turn * math.radians(MAX_YAW_RATE) * dt
            n.manual_yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        # Desired body velocity -> world (PX4 yaw: 0 = north, + = clockwise)
        c, s = math.cos(n.manual_yaw), math.sin(n.manual_yaw)
        vf, vr = fwd * MAX_SPEED, right * MAX_SPEED
        want = [c * vf - s * vr, s * vf + c * vr]
        wn = math.hypot(*want)
        if wn > MAX_SPEED:                          # forward + slide combined: cap the total, not each axis
            want = [w * MAX_SPEED / wn for w in want]
        self._v = [approach(self._v[i], want[i], ACCEL_XY, BRAKE_XY, dt) for i in range(2)]
        self._vz = approach(self._vz, up * ALT_RATE, ACCEL_Z, ACCEL_Z, dt)

        # Advance the target along the commanded velocity, leashed to the drone
        cur, t = n.current_pos, n.target_pos
        t[0] += self._v[0] * dt
        t[1] += self._v[1] * dt
        t[2] -= self._vz * dt                       # NED: up = negative
        dx, dy = t[0] - cur[0], t[1] - cur[1]
        d = math.hypot(dx, dy)
        if d > LEASH_XY:
            t[0] = cur[0] + dx * LEASH_XY / d
            t[1] = cur[1] + dy * LEASH_XY / d
        dz = t[2] - cur[2]
        if abs(dz) > LEASH_Z:
            t[2] = cur[2] + math.copysign(LEASH_Z, dz)
        if t[2] > -MIN_ALT_M:
            t[2] = -MIN_ALT_M
        n.manual_vel = [self._v[0], self._v[1], -self._vz]

        stopped = not active and math.hypot(*self._v) < 0.05 and abs(self._vz) < 0.05
        if stopped:
            self._idle += dt
            n.manual_vel = [0.0, 0.0, 0.0]
            if self._idle > IDLE_RESET_S:
                self._burst = False

    # ---------- camera ----------
    def _camera(self, pitch_in, yaw_in, recentre, dt):
        g = self.node.gimbal_cmd
        if recentre:
            g[0], g[1] = GIMBAL_HOME
            return
        g[0] = min(GIMBAL_PITCH_RANGE[1], max(GIMBAL_PITCH_RANGE[0], g[0] + pitch_in * GIMBAL_RATE * dt))
        g[1] = min(GIMBAL_YAW_RANGE[1], max(GIMBAL_YAW_RANGE[0], g[1] + yaw_in * GIMBAL_RATE * dt))

    # ---------- one poll ----------
    def step(self, j, prev, dt):
        """One polling cycle. Returns the current button states."""
        buttons = [j.get_button(i) for i in range(j.get_numbuttons())]
        if getattr(self.node, "block_ended", False):
            return buttons                         # block over: ignore all input

        def held(i):
            return i < len(buttons) and bool(buttons[i])

        def pressed(i):
            return held(i) and not (i < len(prev) and prev[i])

        lb, rb = held(BTN_LB), held(BTN_RB)
        now = time.time()

        # Emergency: both bumpers held EMERGENCY_HOLD_S
        if lb and rb:
            self._both_since = (self._both_since or 0.0) + dt     # held time (loop clock)
            if not self._emergency_sent and self._both_since >= EMERGENCY_HOLD_S - 1e-6:
                self._emergency_sent = True
                self._log("[CONTROLLER] LB+RB held → emergency return home and land")
                if hasattr(self.node, "emergency_return"):
                    self.node.emergency_return()
                else:
                    self._send(EMERGENCY_LAND)
                return buttons
        else:
            self._both_since = None
            self._emergency_sent = False

        fwd = -shape(self._axis(j, AX_LEFT_Y))     # pygame Y is negative when pushed up
        turn = shape(self._axis(j, AX_LEFT_X))
        right = self._trigger(j, AX_RT) - self._trigger(j, AX_LT)
        up = 0.0 if (lb and rb) else (1.0 if rb else (-1.0 if lb else 0.0))
        cam_pitch = shape(self._axis(j, AX_RIGHT_Y))   # stick down = look further down
        cam_yaw = shape(self._axis(j, AX_RIGHT_X))

        self._fly(fwd, right, up, turn, dt)
        self._camera(cam_pitch, cam_yaw, pressed(BTN_R3), dt)

        # A1: camera stick and any other button count as operator input
        other_press = any(pressed(i) for i in range(len(buttons)) if i not in NO_INPUT_RESET)
        # (If the agent is flying, the pilot's loop sees the fresh input time and
        # hands control back, the same path the old button handling used.)
        if (cam_pitch or cam_yaw or other_press) and hasattr(self.node, "last_key_time"):
            self.node.last_key_time = now

        self._last = {"lx": round(self._axis(j, AX_LEFT_X), 3), "ly": round(self._axis(j, AX_LEFT_Y), 3),
                      "rx": round(self._axis(j, AX_RIGHT_X), 3), "ry": round(self._axis(j, AX_RIGHT_Y), 3),
                      "lt": round(self._trigger(j, AX_LT), 3), "rt": round(self._trigger(j, AX_RT), 3),
                      "buttons": [i for i, b in enumerate(buttons) if b]}
        self._tick += 1
        if self._tick % self._log_every == 0:
            self._write_log()
        return buttons

    # ---------- controls log ----------
    def _write_log(self):
        n = self.node
        t0 = getattr(n, "flight_start_time", None)
        if t0 is None:
            return
        try:
            if self._log_path is None:
                os.makedirs(DATASET_DIR, exist_ok=True)
                sid = datetime.fromtimestamp(t0).strftime("%Y%m%d_%H%M%S")
                self._log_path = os.path.join(DATASET_DIR, f"controls_{sid}.jsonl")
            mv = getattr(n, "manual_vel", None)
            rec = {"t_s": round(time.time() - t0, 3),
                   "state": getattr(getattr(n, "drone_state", None), "value", None),
                   **self._last,
                   "cmd_vel_ned": [round(v, 3) for v in mv] if mv else None,
                   "yaw_sp_deg": round(math.degrees(n.manual_yaw), 2) if getattr(n, "manual_yaw", None) is not None else None,
                   "gimbal_pitch_deg": round(n.gimbal_cmd[0], 2), "gimbal_yaw_deg": round(n.gimbal_cmd[1], 2),
                   "pos_ned": [round(float(v), 2) for v in n.current_pos] if hasattr(n, "current_pos") else None}
            with open(self._log_path, "a") as f:
                f.write(json.dumps(rec) + "\n")
        except Exception as e:
            self._log(f"[CONTROLLER] log error: {e!r}")

    # ---------- thread ----------
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

    def _loop(self):
        j = self._open()
        if j is None:
            return
        prev = [0] * j.get_numbuttons()
        last = time.time()
        while self.running:
            if self._joystick is None:
                pygame.event.pump()
            now = time.time()
            dt = min(0.2, max(1e-3, now - last))   # measured, bounded
            last = now
            prev = self.step(j, prev, dt)
            time.sleep(1.0 / POLL_HZ)


def _detect():
    pygame.display.init(); pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit("No controller found.")
    j = pygame.joystick.Joystick(0); j.init()
    print(f"{j.get_name()}: {j.get_numaxes()} axes, {j.get_numbuttons()} buttons. Ctrl+C to exit.")
    print("Move ONE stick/trigger at a time; note which axis number changes.")
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
