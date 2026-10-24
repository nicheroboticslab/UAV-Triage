"""
F2: the agent's prompt for the flood-call study (replaces the sector/target/
comms/EKF prompt when a call set is loaded). Wired 24 Oct (Ben: rework the
prompt for no-fly zones and an optimised arrival order).

The LLM still picks one intent per decision; WHERE intents go is deterministic:
  3 SEARCH_NEXT_PRIORITY  -> next call of the cost-reward plan (agent_planner.py)
  4 INSPECT_TARGET_REPORT -> nearest open call
  2 CONTINUE_ASSIGNED_TASK-> the call it is heading to
  6 RETURN_TO_BASE, 7 ABORT_AND_SAFE (land now), 1 HOLD_SAFE
  5 COMMS_REACQUIRE is not offered (no comms model) and is treated as 1.
The prompt shows the calls (call text, dispatch rank, distance/bearing along the
no-fly-aware path, inspected, reported by the operator), the plan, battery with
the cost to fly home, Condition and the no-fly zones. Reasoning is requested and
logged (H4) but never shown to the participant (E9).
"""
import math

COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
OFFERED = {
    1: ("HOLD_SAFE", "Hold: circle at the current position."),
    2: ("CONTINUE_ASSIGNED_TASK", "Continue to the call you are heading to."),
    3: ("SEARCH_NEXT_PRIORITY", "Go to the next call of the route plan below."),
    4: ("INSPECT_TARGET_REPORT", "Go to the nearest open call."),
    6: ("RETURN_TO_BASE", "Return to home base and land."),
    7: ("ABORT_AND_SAFE", "Land now where you are (only if home cannot be reached)."),
}
NOT_OFFERED = {5: 1}


def bearing(de, dn):
    return COMPASS[int(round((math.degrees(math.atan2(de, dn)) % 360.0) / 45.0)) % 8]


def build_prompt(ctx):
    """ctx keys: pos_en (east, north), battery_pct, home_cost_pct, condition_pct,
    in_zone, heading_to, idle_s, holding_for (call id or None),
    calls [{id, text, rank, e, n, path_m, visited, reported}],
    zones [{id, name, east_m, north_m, radius_m}], plan {order, time_s, battery_end_pct},
    events (str), reserve_pct, reserve_per_100m."""
    e, n = ctx["pos_en"]
    lines = []
    for c in sorted(ctx["calls"], key=lambda c: c["rank"]):
        st = "REPORTED by operator" if c["reported"] else ("inspected, not yet reported" if c["visited"]
                                                            else "open")
        lines.append(f'  {c["id"]} (dispatch rank {c["rank"]}): "{c["text"]}" - {c["path_m"]:.0f} m '
                     f'{bearing(c["e"] - e, c["n"] - n)} - {st}')
    zl = []
    for z in ctx["zones"]:
        d = max(0.0, math.hypot(z["east_m"] - e, z["north_m"] - n) - z["radius_m"])
        zl.append(f'  {z["id"]} {z["name"]}: radius {z["radius_m"]:.0f} m, edge {d:.0f} m '
                  f'{bearing(z["east_m"] - e, z["north_m"] - n)}')
    plan = ctx["plan"]
    plan_txt = (" -> ".join(plan["order"]) + " -> home" if plan["order"] else "no call can be served; go home")
    zone_now = (f"INSIDE no-fly zone {ctx['in_zone']} - leave it now" if ctx.get("in_zone")
                else "not in a no-fly zone")
    hold = (f"\nHolding over {ctx['holding_for']} until the operator files its report."
            if ctx.get("holding_for") else "")
    menu = "\n".join(f"{k} - {v[0]}: {v[1]}" for k, v in OFFERED.items())
    d_home = math.hypot(e, n)
    return f"""You are the autonomous agent flying a drone for a flood emergency response.
The operator has not given any input for {ctx['idle_s']:.0f} s, so you are flying.
Your job: bring the drone to the 911 call locations so the operator can see each scene and
file a report. Stay over a call until its report is filed, then go on. Life safety first.

=== VEHICLE ===
Home base       : {d_home:.0f} m {bearing(-e, -n)}
Battery         : {ctx['battery_pct']:.0f} %; flying home from here needs about {ctx['home_cost_pct']:.0f} %.
                  Below {ctx['reserve_pct']:.0f} % each 100 m flown costs {ctx['reserve_per_100m']:.0f} %. At 0 % the drone lands where it is.
Condition       : {ctx['condition_pct']:.0f} % ({zone_now}). Collisions and no-fly zones reduce it; at 0 % the drone is lost.
Heading to      : {ctx['heading_to'] or 'nothing'}{hold}

=== CALLS (distance along the flyable path, around no-fly zones) ===
{chr(10).join(lines)}
Dispatch rank 1 is the call the dispatcher judged most urgent; the scene may differ from the call.

=== ROUTE PLAN (cost-reward planner: urgency weighted by arrival time, no-fly detours, battery to get home) ===
{plan_txt}   (about {plan['time_s']:.0f} s, battery at home about {plan['battery_end_pct']:.0f} %)

=== NO-FLY ZONES (never enter; your flight path goes around them) ===
{chr(10).join(zl) if zl else '  none'}

=== RECENT EVENTS ===
{ctx.get('events') or '  none'}

=== ACTIONS ===
{menu}

RULES:
- Normally follow the route plan (3). Use 2 to keep going to the call you are heading to.
- Do not leave a call that is not yet reported unless the battery requires it.
- Return home (6) when the battery is close to what the flight home needs.
- Land now (7) only if home cannot be reached.

Respond ONLY with valid JSON:
{{"intent":<1, 2, 3, 4, 6 or 7>,"reasoning":"<one short paragraph>","confidence":<0-100>,"next_checkpoint":"<when to reconsider>"}}"""
