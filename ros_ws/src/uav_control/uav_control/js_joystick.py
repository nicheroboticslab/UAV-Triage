"""
Linux joystick reader that ignores window focus (Ben, 24 Oct: the controller must
always work, even while another, unrelated window has keyboard focus).

Reads /dev/input/jsN (Linux joystick API, joydev) in a background thread. It has
nothing to do with windows or SDL, so focus can never block it. Implements the
small part of pygame.joystick.Joystick that controller_input.py uses:
get_axis, get_button, get_hat, get_numaxes, get_numbuttons, get_name, get_init.

Numbering: joydev orders axes by evdev code (X, Y, Z, RX, RY, RZ, HAT0X, HAT0Y)
and buttons by key code (BTN_A, B, X, Y, TL, TR, SELECT, START, MODE, THUMBL,
THUMBR, then the D-pad buttons of wireless pads), the same order SDL2 uses for
these axes and buttons on xpad. Check with:  python3 controller_input.py --detect-js
Axis values are scaled to -1..1 as in pygame.
"""
import fcntl
import glob
import os
import struct
import threading

JS_EVENT = struct.Struct("IhBB")          # time (ms), value, type, number
JS_BUTTON, JS_AXIS, JS_INIT = 0x01, 0x02, 0x80
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12


def _jsiocgname(n):
    return 0x80006A13 + (n << 16)


def devices():
    return sorted(glob.glob("/dev/input/js*"))


class JsJoystick:
    def __init__(self, path):
        self.path = path
        self.fd = os.open(path, os.O_RDONLY)
        buf = bytearray(1)
        fcntl.ioctl(self.fd, JSIOCGAXES, buf)
        self.n_axes = buf[0]
        fcntl.ioctl(self.fd, JSIOCGBUTTONS, buf)
        self.n_buttons = buf[0]
        name = bytearray(128)
        try:
            fcntl.ioctl(self.fd, _jsiocgname(len(name)), name)
            self.name = name.split(b"\0", 1)[0].decode(errors="replace")
        except OSError:
            self.name = os.path.basename(path)
        self.axes = [0.0] * self.n_axes
        self.buttons = [0] * self.n_buttons
        self.alive = True
        self.lock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        try:
            while self.alive:
                data = os.read(self.fd, JS_EVENT.size * 16)
                if not data:
                    break
                for k in range(0, len(data) - JS_EVENT.size + 1, JS_EVENT.size):
                    _, value, typ, num = JS_EVENT.unpack_from(data, k)
                    typ &= ~JS_INIT
                    with self.lock:
                        if typ == JS_AXIS and num < self.n_axes:
                            self.axes[num] = max(-1.0, value / 32767.0)
                        elif typ == JS_BUTTON and num < self.n_buttons:
                            self.buttons[num] = 1 if value else 0
        except OSError:
            pass
        self.alive = False
        try:
            os.close(self.fd)
        except OSError:
            pass

    # --- pygame.joystick.Joystick subset
    def init(self):
        pass

    def get_init(self):
        return self.alive

    def get_name(self):
        return self.name

    def get_numaxes(self):
        return self.n_axes

    def get_numbuttons(self):
        return self.n_buttons

    def get_numhats(self):
        return 1 if self.n_axes >= 8 else 0

    def get_axis(self, i):
        with self.lock:
            return self.axes[i] if i < self.n_axes else 0.0

    def get_button(self, i):
        with self.lock:
            return self.buttons[i] if i < self.n_buttons else 0

    def get_hat(self, i):
        """D-pad on pads that report it as axes 6/7 (wired xpad): (x, y), y up = +1."""
        if self.n_axes < 8:
            return (0, 0)
        x, y = self.get_axis(6), self.get_axis(7)
        return (int(round(x)), -int(round(y)))

    def quit(self):
        self.alive = False


def open_first():
    """First readable /dev/input/js* device, or None."""
    for p in devices():
        try:
            return JsJoystick(p)
        except OSError:
            continue
    return None
