"""
Agent call routing (F2/G3, Ben 24 Oct): a deterministic cost-reward planner that
picks the order in which the agent visits the open calls.

Ben: "optimise the order of arrival ... the agent should probably use a
cost-reward system to route. The order I gave [dispatch order] is likely not
appropriate." and "the agent does not fly away until after the report for a
call is filed".

Model (all settings in agent.json, defaults below):
  - Reward of serving call i = weight of its dispatch rank (rank 1 = most urgent
    by the dispatcher), discounted by how late the drone gets there:
        value_i = w(rank_i) * 0.5 ** (t_arrive_i / half_life_s)
    The dispatcher's ranking is the only urgency information the agent has (it
    never sees the ground truth). Calls the operator has already reported have
    no remaining value.
  - Cost = time: legs flown at agent cruise along the no-fly-aware path
    (hazards.ConditionMonitor.detour), plus an expected hold of hold_est_s at each
    call while the operator files the report.
  - Feasibility: after the last call the drone must still be able to fly home
    with return_margin_pct to spare, using the vehicle's own battery model
    (Zeng Eq. 6 at cruise, hover power while holding, reserve rule below
    reserve_pct).
  - Plan = the subset and order of open calls (all permutations of all subsets;
    at most 5 calls -> 325 plans) with the highest total value; ties -> shorter
    total time. Empty plan -> go home.
Deterministic: same state in, same plan out (F1).
"""
import itertools
import json
import math
import os

REPO = os.path.expanduser(os.environ.get("UAV_REPO", "~/UAV-Triage"))
DEFAULTS = {
    "cruise_ms": 14.0,
    "rank_weights": {"1": 5.0, "2": 4.0, "3": 3.0, "4": 2.0, "5": 1.0},
    "half_life_s": 300.0,        # value of reaching a call halves every 5 min
    "hold_est_s": 45.0,          # expected hold per call for the report (decisions log 6.5)
    "max_hold_s": 120.0,         # agent waits at most this long for the report, then moves on
    "min_hold_s": 5.0,           # call already reported when the agent arrives: brief look, then go
    "return_margin_pct": 3.0,    # battery kept on top of the cost to fly home
    "arrive_radius_m": 25.0,     # a call counts as visited when the vehicle is this close
    "leg_climb_m": 35.0,         # each leg: climb from call altitude (10 m) to transit (45 m) and back down
    "climb_ms": 5.0,             # vertical speeds for that climb / descent (PX4 caps 6 up / 5 down; less for acceleration)
    "descent_ms": 4.0,
    "hold_radius_m": 150.0,      # takeover within this distance of an unreported call: hold for the report
                                 # (same radius as the portal's active call)
}


def load_config(path=None):
    cfg = json.loads(json.dumps(DEFAULTS))
    try:
        cfg.update({k: v for k, v in json.load(open(path or os.path.join(REPO, "agent.json"))).items()
                    if not k.startswith("_")})
    except FileNotFoundError:
        pass
    return cfg


class Energy:
    """Battery % used per metre at cruise and per second of hover, with the reserve
    rule of battery.json (below reserve_pct: reserve_pct_per_100m per 100 m flown)."""

    def __init__(self, batt_cfg, cruise_ms):
        from battery_model import PowerModel
        pm = PowerModel(batt_cfg["vehicle"])
        e = batt_cfg["battery"]["usable_energy_wh"]
        self.pct_per_m = pm.level(cruise_ms) / cruise_ms / 3600.0 / e * 100.0
        self.pct_per_s_hover = pm.level(0.0) / 3600.0 / e * 100.0
        g = batt_cfg["gate"]
        self.reserve = float(g.get("reserve_pct") or 0.0)
        self.e_wh = e
        self.mass = batt_cfg["vehicle"]["mass_kg"]
        self.per100 = float(g.get("reserve_pct_per_100m", 1.0))

    def after_flight(self, pct, metres):
        above = max(0.0, pct - self.reserve)
        m_above = above / self.pct_per_m
        if metres <= m_above:
            return pct - metres * self.pct_per_m
        return pct - above - (metres - m_above) / 100.0 * self.per100

    def after_hover(self, pct, seconds):
        return pct - seconds * self.pct_per_s_hover   # hover drains at the model rate in reserve too

    def after_climb(self, pct, climb_m, climb_ms, descent_ms):
        """Climb to transit altitude and descend at the goal: potential energy (m g h, as in
        the battery model) plus hover power for the vertical time."""
        wh = self.mass * 9.81 * climb_m / 3600.0
        t = climb_m / climb_ms + climb_m / descent_ms
        return pct - wh / self.e_wh * 100.0 - t * self.pct_per_s_hover, t

    def home_cost(self, pct, metres):
        return pct - self.after_flight(pct, metres)


class Planner:
    def __init__(self, cond, batt_cfg, cfg=None):
        self.cond = cond                       # hazards.ConditionMonitor (detours); may be None
        self.cfg = cfg or load_config()
        self.energy = Energy(batt_cfg, self.cfg["cruise_ms"])
        self._len_cache = {}

    def path(self, a, b):
        """No-fly-aware path from a to b, both (east, north): list of points incl. b."""
        via = self.cond.detour(a, b) if (self.cond is not None and self.cond.ok) else []
        return list(via) + [b]

    def length(self, a, b):
        key = (round(a[0], 1), round(a[1], 1), round(b[0], 1), round(b[1], 1))
        if key not in self._len_cache:
            pts = [a] + self.path(a, b)
            self._len_cache[key] = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        return self._len_cache[key]

    def plan(self, pos_en, battery_pct, calls, elapsed_s=0.0):
        """calls: [{id, e, n, rank, done}] (done = reported by the operator).
        Returns {"order": [ids], "value", "time_s", "battery_end_pct", "home_cost_pct", "evaluated"}."""
        c = self.cfg
        open_calls = [x for x in calls if not x.get("done")]
        best = None
        n = 0
        home = (0.0, 0.0)
        for k in range(len(open_calls), 0, -1):
            for perm in itertools.permutations(open_calls, k):
                n += 1
                t, pct, val, here = elapsed_s, battery_pct, 0.0, pos_en
                for x in perm:
                    d = self.length(here, (x["e"], x["n"]))
                    pct, tv = self.energy.after_climb(pct, c["leg_climb_m"], c["climb_ms"], c["descent_ms"])
                    t += d / c["cruise_ms"] + tv
                    pct = self.energy.after_flight(pct, d)
                    w = float(c["rank_weights"].get(str(x["rank"]), 1.0))
                    val += w * 0.5 ** ((t - elapsed_s) / c["half_life_s"])
                    t += c["hold_est_s"]
                    pct = self.energy.after_hover(pct, c["hold_est_s"])
                    here = (x["e"], x["n"])
                d_home = self.length(here, home)
                end, _ = self.energy.after_climb(pct, c["leg_climb_m"], c["climb_ms"], c["descent_ms"])
                end = self.energy.after_flight(end, d_home)
                if end < c["return_margin_pct"]:
                    continue
                key = (round(val, 6), -(t - elapsed_s))
                if best is None or key > best[0]:
                    best = (key, [x["id"] for x in perm], val, t - elapsed_s, end)
        d_now = self.length(pos_en, home)
        out = {"home_cost_pct": round(self.energy.home_cost(battery_pct, d_now), 2), "evaluated": n}
        if best is None:
            out.update({"order": [], "value": 0.0, "time_s": round(d_now / c["cruise_ms"], 1),
                        "battery_end_pct": round(self.energy.after_flight(battery_pct, d_now), 2)})
        else:
            out.update({"order": best[1], "value": round(best[2], 3), "time_s": round(best[3], 1),
                        "battery_end_pct": round(best[4], 2)})
        return out

    def gate(self, pos_en, battery_pct):
        """Deterministic battery gate before any LLM query: 7 land now (home out of
        reach), 6 return home (battery at the cost home + margin), else None."""
        need = self.energy.home_cost(battery_pct, self.length(pos_en, (0.0, 0.0)))
        if battery_pct < need:
            return 7
        if battery_pct <= need + self.cfg["return_margin_pct"]:
            return 6
        return None
