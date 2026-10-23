"""
Helper for scenario.json.

    python3 scenario_tool.py --show   # table of sectors: metres from home, legs, sweep time
    python3 scenario_tool.py --env    # prints PX4_HOME_* exports (used by the launcher)
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
