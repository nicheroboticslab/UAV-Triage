"""
Helper for scenario.json and the call sets.

    python3 scenario_tool.py --show      # table of sectors: metres from home, legs, sweep time
    python3 scenario_tool.py --env       # prints PX4_HOME_* exports (used by the launcher)
    python3 scenario_tool.py --pool      # call pool: distance from home, missing coordinates (C3)
    python3 scenario_tool.py --set S1    # validate one call set; optimal route, time, energy (C4)
    python3 scenario_tool.py --sets      # compare all sets in calls/sets for matching (H3)
"""
import json, math, os, sys

PATH = os.environ.get("UAV_SCENARIO", os.path.expanduser("~/UAV-Triage/scenario.json"))
sc = json.load(open(PATH))
h = sc["home"]

if "--env" in sys.argv:
    print(f'export PX4_HOME_LAT={h["lat"]}')
    print(f'export PX4_HOME_LON={h["lon"]}')
    print(f'export PX4_HOME_ALT={h.get("alt_m", 0)}')
    sys.exit(0)

if any(a in sys.argv for a in ("--pool", "--set", "--sets")):
    sys.path.insert(0, os.path.expanduser("~/UAV-Triage/ros_ws/src/uav_control/uav_control"))
    import call_sets
    from battery_model import PowerModel, load_config
    bcfg, _ = load_config()
    power = PowerModel(bcfg["vehicle"])
    usable = bcfg["battery"]["usable_energy_wh"]
    cruise_alt = float(sc.get("cruise_alt_m", 10))

    if "--pool" in sys.argv:
        pool = call_sets.load_pool()
        print(f'Pool: {call_sets.POOL_PATH} ({len(pool)} locations; within 1.5 km of home)')
        print(f'{"ID":<4} {"LABEL":<34} {"TYPE":<24} {"FROM_HOME":>9}')
        for lid, loc in pool.items():
            if loc.get("lat") is None:
                dist = "no coords"
            else:
                n, e, _ = call_sets.ned(loc["lat"], loc["lon"], h, 0)
                r = math.hypot(n, e)
                dist = f'{r:7.0f} m' + ("  > 1.5 km!" if r > 1500 else "") \
                       + ("  PLACEHOLDER" if loc.get("placeholder") else "")
            print(f'{lid:<4} {loc.get("label",""):<34} {str(loc.get("call_type") or "-"):<24} {dist}')
        sys.exit(0)

    def report(set_id):
        cs, calls = call_sets.resolve(set_id, h, cruise_alt)
        m = call_sets.route_metrics(calls, power=power)
        m["battery_used_pct"] = round(100 * m["energy_wh"] / usable, 1)
        return cs, calls, m

    if "--set" in sys.argv:
        sid = sys.argv[sys.argv.index("--set") + 1]
        cs, calls, m = report(sid)
        print(f'Call set {sid}: {len(calls)} calls')
        for c in sorted(calls, key=lambda c: c["priority"]):
            print(f'  P{c["priority"]} {c["call_id"]:<4} {c["location"]:<4} {c["label"]:<34} {str(c["call_type"] or "-"):<24} '
                  f'{str(c["scenario_code"]):<14} {math.hypot(c["pos"][0], c["pos"][1]):6.0f} m'
                  + ("  PLACEHOLDER" if c["placeholder"] else ""))
        print(f'Optimal order: home -> {" -> ".join(m["optimal_order"])} -> home')
        print(f'Route {m["route_m"]/1000:.2f} km | transit {m["transit_s"]:.0f} s at 14 m/s + 45 s per call '
              f'= {m["total_s"]/60:.1f} min | energy {m["energy_wh"]} Wh ({m["battery_used_pct"]}% of battery)')
        sys.exit(0)

    if "--sets" in sys.argv:
        ids = sorted(f[:-5] for f in os.listdir(call_sets.SET_DIR) if f.endswith(".json") and f not in ("TEMPLATE.json", "DEV.json", "WATERTEST.json"))
        rows = []
        for sid in ids:
            try:
                rows.append((sid,) + report(sid)[1:])
            except ValueError as e:
                print(f'{sid}: INVALID - {e}')
        if rows:
            exp = [r for r in rows if not r[0].upper().startswith("T")]   # training set excluded from matching
            mean = sum(r[2]["total_s"] for r in exp) / len(exp) if exp else 0
            print(f'{"SET":<6} {"CALLS":>5} {"ROUTE_km":>8} {"TIME_min":>8} {"vs_mean":>8} {"BATT_%":>6}  TYPES')
            for sid, calls, m in rows:
                dev = (m["total_s"] - mean) / mean * 100 if mean and sid in [r[0] for r in exp] else float("nan")
                print(f'{sid:<6} {len(calls):>5} {m["route_m"]/1000:>8.2f} {m["total_s"]/60:>8.1f} {dev:>+7.1f}% '
                      f'{m["battery_used_pct"]:>6}  {",".join(m["call_types"])}')
            # locations shared between sets (a participant flies every set)
            used = {}
            for sid, calls, _ in rows:
                for c in calls:
                    used.setdefault(c["location"], []).append(sid)
            dup = {k: v for k, v in used.items() if len(v) > 1}
            print("Locations reused across sets:", dup if dup else "none")
        sys.exit(0)

k = 111320.0; kl = k * math.cos(math.radians(h["lat"]))
def ned(p): return (p["lat"] - h["lat"]) * k, (p["lon"] - h["lon"]) * kl

print(f'Scenario: {PATH}')
print(f'Home: {h.get("name","")} {h["lat"]}, {h["lon"]}  (ground {h.get("alt_m",0)} m)\n')
print(f'{"PRI":>3}  {"NAME":<9} {"LABEL":<24} {"NORTH_m":>8} {"EAST_m":>8} {"FROM_HOME":>9}')
secs = sorted(sc["sectors"], key=lambda s: s["priority"])
prev = (0.0, 0.0); total = 0.0
for s in secs:
    n, e = ned(s); total += math.hypot(n - prev[0], e - prev[1]); prev = (n, e)
    print(f'{s["priority"]:>3}  {s["name"]:<9} {s.get("label",""):<24} {n:>8.0f} {e:>8.0f} {math.hypot(n,e):>8.0f}m')
total += math.hypot(*prev)
print(f'\nSweep route (home -> sectors by priority -> home): {total/1000:.1f} km'
      f'  ~{total/11.5/60:.0f} min at 11.5 m/s')
nfz = sc.get("no_fly_zones", {})
print(f'No-fly zones: {len(nfz.get("circles",[]))} circles, {len(nfz.get("polygons",[]))} polygons')
