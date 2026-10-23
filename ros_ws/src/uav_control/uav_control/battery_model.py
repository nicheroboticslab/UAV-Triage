"""
Deterministic battery model (requirements D2, D3, D6).

Power: Zeng, Xu & Zhang (2019) Eq. 6 rotary-wing model, applied per rotor
(four rotors, W/4 each) plus a fuselage parasite term, calibrated to the
DJI Matrice 30T (decisions log 3.3-3.4). With the parameters in
battery.json the curve gives hover 434 W, minimum 384 W at 8.1 m/s,
505 W at 15 m/s, 1024 W at 23 m/s, maximum-range speed 14.4 m/s.
    https://arxiv.org/abs/1804.02238
Extensions (decisions log 4.4-4.5): airspeed = ground velocity - wind;
climb power m*g*v_climb added when climbing (no regeneration on descent).

Drain: time-stepped as in Traub (2016): current = P / V, charge += i*dt.
    https://www.mdpi.com/2313-0105/2/2/17
Voltage model: "constant" only (V = nominal), so displayed % equals remaining
energy and drain follows the power profile. A discharge curve (Tremblay &
Dessaint 2009, decisions log 4.6) plugs into Battery.voltage() once sourced
datasheet points are chosen; with it, current rises as voltage sags and the
late-flight acceleration of the % appears.

Identical inputs give identical outputs: no randomness, fixed step.
"""
import json
import math
import os

DEFAULT_PATH = os.path.expanduser("~/UAV-Triage/battery.json")
G = 9.81


def load_config(path=None):
    path = path or os.environ.get("UAV_BATTERY", DEFAULT_PATH)
    with open(path) as f:
        return json.load(f), path


class PowerModel:
    def __init__(self, v):
        self.rho = v["air_density"]
        self.R = v["rotor_radius_m"]
        self.n = v["rotors"]
        self.m = v["mass_kg"]
        self.k = v["induced_correction_k"]
        self.delta = v["profile_drag_coeff"]
        self.Ut = v["tip_speed_ms"]
        self.s = v["rotor_solidity"]
        self.Sfp = v["flat_plate_area_m2"]
        A = math.pi * self.R ** 2
        w = self.m * G / self.n
        self.P0 = self.delta / 8 * self.rho * self.s * A * self.Ut ** 3
        self.Pi = (1 + self.k) * w ** 1.5 / math.sqrt(2 * self.rho * A)
        self.v0 = math.sqrt(w / (2 * self.rho * A))

    def level(self, V):
        """Zeng Eq. 6 level-flight power (W) at airspeed V (m/s), all rotors + fuselage."""
        rotor = (self.P0 * (1 + 3 * V ** 2 / self.Ut ** 2)
                 + self.Pi * math.sqrt(math.sqrt(1 + V ** 4 / (4 * self.v0 ** 4))
                                       - V ** 2 / (2 * self.v0 ** 2)))
        return self.n * rotor + 0.5 * self.rho * self.Sfp * V ** 3

    def total(self, vel_ned, wind_ned=(0.0, 0.0)):
        vx = vel_ned[0] - wind_ned[0]
        vy = vel_ned[1] - wind_ned[1]
        V = math.hypot(vx, vy)
        climb = max(0.0, -vel_ned[2])          # NED: negative z velocity = climbing
        return self.level(V) + self.m * G * climb, V


class Battery:
    def __init__(self, cfg):
        b = cfg["battery"]
        self.energy_wh = b["usable_energy_wh"]
        self.v_nom = b["nominal_voltage_v"]
        self.q_ah = self.energy_wh / self.v_nom
        self.model = b.get("voltage_model", "constant")
        self.drawn_ah = 0.0
        self.drawn_wh = 0.0
        if self.model != "constant":
            raise ValueError("battery.json: only voltage_model 'constant' is implemented; a "
                             "discharge-curve model needs sourced datasheet points first")

    def voltage(self, current):
        return self.v_nom

    def step(self, power_w, dt):
        v = self.voltage(power_w / self.v_nom)
        current = power_w / max(v, 1e-3)
        self.drawn_ah += current * dt / 3600.0
        self.drawn_wh += power_w * dt / 3600.0
        return v, current

    @property
    def pct(self):
        return max(0.0, 100.0 * (1.0 - self.drawn_ah / self.q_ah))


class BatterySystem:
    """Power model + battery + gate logic. Call step() at a fixed dt."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.power = PowerModel(cfg["vehicle"])
        self.battery = Battery(cfg)
        g = cfg["gate"]
        self.warn_pct = g["warn_pct"]
        self.return_pct = g["return_pct"]
        self.wind = tuple(cfg.get("wind_ned_ms", [0.0, 0.0]))
        self.warned = False
        self.gate_reached = False
        self.last = {"power_w": 0.0, "airspeed_ms": 0.0, "voltage_v": self.battery.v_nom, "current_a": 0.0}

    def step(self, vel_ned, dt, flying=True):
        """Advance one step. Returns events: list of 'warn' / 'gate'."""
        if flying:
            p, V = self.power.total(vel_ned, self.wind)
        else:
            p, V = 0.0, 0.0
        v, i = self.battery.step(p, dt)
        self.last = {"power_w": p, "airspeed_ms": V, "voltage_v": v, "current_a": i}
        events = []
        pct = self.battery.pct
        if not self.warned and pct <= self.warn_pct:
            self.warned = True
            events.append("warn")
        if not self.gate_reached and pct <= self.return_pct:
            self.gate_reached = True
            events.append("gate")
        return events

    @property
    def pct(self):
        return self.battery.pct
