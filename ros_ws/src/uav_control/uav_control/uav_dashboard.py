#!/usr/bin/env python3
"""
UAV Operator Dashboard — Stage 4
=================================
Standalone curses-based live display for the UAV autonomous system.
Run this in a SEPARATE terminal while autonomous_uav_stage4.py is running.

Usage:
    python3 uav_dashboard.py

Reads /tmp/uav_dashboard_state.json written by the ROS node every 0.5s.
Press Q to quit the dashboard (does NOT stop the drone).

What it shows:
  - Drone state, flight time, battery bar, EKF, signal status
  - Last LLM decision: intent, target, confidence, full reasoning
  - Next checkpoint
  - Last 8 flight log entries
  - Post-signal-loss re-entry indicator
  - Dwell countdown when active
"""

import curses
import json
import time
import os
import sys

DASHBOARD_STATE_FILE = "/tmp/uav_dashboard_state.json"
REFRESH_RATE         = 0.5   # seconds between screen refreshes


def battery_bar(pct: float, width: int = 10) -> str:
    """Renders a text battery bar like ████████░░  82.3%"""
    filled  = int(round(pct / 100.0 * width))
    empty   = width - filled
    bar     = "█" * filled + "░" * empty
    return f"{bar}  {pct:5.1f}%"


def wrap_text(text: str, width: int) -> list:
    """Word-wrap a string to fit within width characters. Returns list of lines."""
    words = text.split()
    lines = []
    line  = ""
    for word in words:
        if len(line) + len(word) + (1 if line else 0) <= width:
            line += ("" if not line else " ") + word
        else:
            if line:
                lines.append(line)
            line = word
    if line:
        lines.append(line)
    return lines if lines else [""]


def draw_dashboard(stdscr, state: dict):
    """
    Renders the full dashboard to the curses screen.
    Called every REFRESH_RATE seconds.
    """
    stdscr.erase()
    curses.curs_set(0)

    # Terminal dimensions
    max_y, max_x = stdscr.getmaxyx()
    W = min(max_x - 1, 72)   # Dashboard width — fits most terminals

    # ---- Color pairs ----
    # 1=normal, 2=green, 3=yellow, 4=red, 5=cyan, 6=magenta, 7=white bold
    curses.init_pair(1, curses.COLOR_WHITE,   curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_GREEN,   curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW,  curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED,     curses.COLOR_BLACK)
    curses.init_pair(5, curses.COLOR_CYAN,    curses.COLOR_BLACK)
    curses.init_pair(6, curses.COLOR_MAGENTA, curses.COLOR_BLACK)

    def put(row, col, text, color=1, bold=False):
        """Safe addstr that won't crash on terminal edge"""
        if row >= max_y - 1 or col >= max_x - 1:
            return
        text = text[:max(0, max_x - col - 1)]
        attr = curses.color_pair(color)
        if bold:
            attr |= curses.A_BOLD
        try:
            stdscr.addstr(row, col, text, attr)
        except curses.error:
            pass

    def hline(row, char="─"):
        put(row, 0, "├" + char * (W - 2) + "┤", 5)

    row = 0

    # ═══ HEADER ═══
    put(row, 0, "╔" + "═" * (W - 2) + "╗", 5, bold=True); row += 1
    title = "UAV AUTONOMOUS SYSTEM — LIVE OPERATOR DISPLAY"
    put(row, 0, "║", 5)
    put(row, 2, title.center(W - 4), 7, bold=True)
    put(row, W - 1, "║", 5); row += 1
    put(row, 0, "╠" + "═" * (W - 2) + "╣", 5); row += 1

    # ═══ STATUS ROW ═══
    drone_state   = state.get("drone_state", "---")
    flight_time   = state.get("flight_time", "T+0s")
    battery_pct   = state.get("battery_pct", 0.0)
    battery_crit  = state.get("battery_crit", False)
    battery_low   = state.get("battery_low", False)
    ekf_conf      = state.get("ekf_confidence", 0.0)
    ekf_ok        = state.get("ekf_ok", True)
    signal_lost   = state.get("signal_lost", False)
    loss_count    = state.get("signal_loss_count", 0)
    llm_running   = state.get("llm_running", False)
    post_reentry  = state.get("post_reentry", False)
    dwell_active  = state.get("dwell_active", False)
    dwell_rem     = state.get("dwell_remaining", 0.0)

    # State color
    state_color = 2 if drone_state == "MANUAL" else (3 if "FLYING" in drone_state else 6)
    llm_tag     = "  🧠THINKING" if llm_running else ("  📡RE-ENTRY" if post_reentry else "")
    dwell_tag   = f"  ⏳DWELL {dwell_rem:.0f}s" if dwell_active else ""

    put(row, 0, "║", 5)
    put(row, 2, f"State : ", 1)
    put(row, 10, f"{drone_state}{llm_tag}{dwell_tag}", state_color, bold=True)
    put(row, W - 14, f"Time: {flight_time}", 5)
    put(row, W - 1, "║", 5); row += 1

    # Battery row
    batt_color = 4 if battery_crit else (3 if battery_low else 2)
    put(row, 0, "║", 5)
    put(row, 2, "Battery: ", 1)
    put(row, 11, battery_bar(battery_pct), batt_color, bold=battery_crit)
    put(row, W - 1, "║", 5); row += 1

    # EKF + Signal row
    ekf_color  = 2 if ekf_ok else 4
    ekf_label  = f"{ekf_conf:.2f} {'✓ GOOD' if ekf_conf >= 0.75 else ('⚠ MOD' if ekf_conf >= 0.4 else '✗ CRIT')}"
    sig_color  = 4 if signal_lost else 2
    sig_label  = "✗ LOST" if signal_lost else "✓ OK"
    loss_label = f"  Losses: {loss_count}" if loss_count > 0 else ""

    put(row, 0, "║", 5)
    put(row, 2,  "EKF    : ", 1)
    put(row, 11, ekf_label, ekf_color)
    put(row, 30, "Signal: ", 1)
    put(row, 38, sig_label, sig_color, bold=signal_lost)
    put(row, 46, loss_label, 3 if loss_count >= 2 else 1)
    put(row, W - 1, "║", 5); row += 1

    # Position row
    pos = state.get("position", [0, 0, 0])
    pos_str = f"[{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]"
    current_wp = state.get("current_wp", "None")
    put(row, 0, "║", 5)
    put(row, 2, f"Pos    : {pos_str}    →  {current_wp}", 1)
    put(row, W - 1, "║", 5); row += 1

    hline(row); row += 1

    # ═══ LLM DECISION SECTION ═══
    put(row, 0, "║", 5)
    put(row, 2, "🧠  LAST PILOT DECISION", 6, bold=True)
    put(row, W - 1, "║", 5); row += 1

    intent_num  = state.get("llm_intent_num", 0)
    intent_name = state.get("llm_intent_name", "NONE")
    llm_wp      = state.get("llm_wp_name", state.get("current_wp", "---"))
    confidence  = state.get("llm_confidence", 0)
    conf_label  = state.get("llm_conf_label", "---")
    reasoning   = state.get("llm_reasoning", "No decision yet.")
    checkpoint  = state.get("llm_checkpoint", "")

    conf_color  = 2 if confidence >= 70 else (3 if confidence >= 50 else 4)
    conf_icon   = "✅" if confidence >= 70 else ("⚠️ " if confidence >= 50 else "❌")

    put(row, 0, "║", 5)
    put(row, 2,  "Intent     : ", 1)
    put(row, 15, f"[{intent_num}] {intent_name}", 6, bold=True)
    put(row, W - 1, "║", 5); row += 1

    put(row, 0, "║", 5)
    put(row, 2,  "Target     : ", 1)
    put(row, 15, llm_wp, 5)
    put(row, W - 1, "║", 5); row += 1

    put(row, 0, "║", 5)
    put(row, 2,  "Confidence : ", 1)
    put(row, 15, f"{conf_icon} {confidence}%  {conf_label}", conf_color, bold=True)
    put(row, W - 1, "║", 5); row += 1

    # Reasoning — word wrapped
    put(row, 0, "║", 5)
    put(row, W - 1, "║", 5); row += 1
    put(row, 0, "║", 5)
    put(row, 2, "Reasoning:", 1, bold=True)
    put(row, W - 1, "║", 5); row += 1

    reason_lines = wrap_text(reasoning, W - 6)
    for i, rline in enumerate(reason_lines[:5]):   # max 5 lines of reasoning
        put(row, 0, "║", 5)
        put(row, 4, rline, 1)
        put(row, W - 1, "║", 5); row += 1
        if row >= max_y - 8:
            break

    # Next checkpoint
    if checkpoint:
        put(row, 0, "║", 5)
        put(row, W - 1, "║", 5); row += 1
        put(row, 0, "║", 5)
        put(row, 2, "Next check : ", 1)
        put(row, 15, checkpoint[:W - 17], 3)
        put(row, W - 1, "║", 5); row += 1

    hline(row); row += 1

    # ═══ FLIGHT LOG SECTION ═══
    if row < max_y - 6:
        put(row, 0, "║", 5)
        put(row, 2, "📋  FLIGHT LOG", 6, bold=True)
        put(row, W - 1, "║", 5); row += 1

        event_log = state.get("event_log", [])
        if not event_log:
            put(row, 0, "║", 5)
            put(row, 4, "No events yet.", 1)
            put(row, W - 1, "║", 5); row += 1
        else:
            for entry in event_log:
                if row >= max_y - 3:
                    break
                put(row, 0, "║", 5)
                # Color-code key event types
                color = 1
                if "CRITICAL" in entry or "ABORT" in entry or "LOST" in entry:
                    color = 4
                elif "WARNING" in entry or "LOW" in entry or "⚠" in entry:
                    color = 3
                elif "LLM decided" in entry or "ARRIVED" in entry or "regained" in entry:
                    color = 2
                elif "TEST" in entry:
                    color = 6
                put(row, 2, entry[:W - 4], color)
                put(row, W - 1, "║", 5); row += 1

    # ═══ FOOTER ═══
    if row < max_y - 2:
        put(row, 0, "╚" + "═" * (W - 2) + "╝", 5); row += 1
    if row < max_y - 1:
        age = time.time() - state.get("timestamp", time.time())
        footer = f" Press Q to quit  |  Last update: {age:.1f}s ago  |  Dashboard v4 "
        put(row, 0, footer.center(W), 5)

    stdscr.refresh()


def load_state() -> dict:
    """Load state from JSON file. Returns empty dict if file not ready yet."""
    try:
        with open(DASHBOARD_STATE_FILE, 'r') as f:
            return json.load(f)
    except Exception:
        return {}


def main_loop(stdscr):
    """Main curses loop"""
    stdscr.nodelay(True)   # Non-blocking key input
    stdscr.timeout(int(REFRESH_RATE * 1000))

    # Check for color support
    if not curses.has_colors():
        stdscr.addstr(0, 0, "Terminal doesn't support colors. Try a different terminal.")
        stdscr.refresh()
        time.sleep(3)
        return

    curses.start_color()
    curses.use_default_colors()

    last_state = {}

    while True:
        # Check for Q to quit
        try:
            ch = stdscr.getch()
            if ch in (ord('q'), ord('Q')):
                break
        except Exception:
            pass

        # Load latest state
        state = load_state()
        if state:
            last_state = state
        elif not last_state:
            # No data yet — show waiting screen
            stdscr.erase()
            stdscr.addstr(0, 0, "Waiting for UAV system to start...")
            stdscr.addstr(1, 0, f"Looking for: {DASHBOARD_STATE_FILE}")
            stdscr.addstr(2, 0, "Make sure autonomous_uav_stage4.py is running.")
            stdscr.addstr(4, 0, "Press Q to quit.")
            stdscr.refresh()
            time.sleep(REFRESH_RATE)
            continue

        # Render dashboard
        try:
            draw_dashboard(stdscr, last_state)
        except Exception as e:
            stdscr.erase()
            stdscr.addstr(0, 0, f"Display error: {e}")
            stdscr.addstr(1, 0, "Terminal may be too small. Try resizing.")
            stdscr.refresh()

        time.sleep(REFRESH_RATE)


def main():
    print(f"UAV Dashboard — looking for state file: {DASHBOARD_STATE_FILE}")
    print("Starting in 1 second... Press Q inside dashboard to quit.\n")
    time.sleep(1.0)

    try:
        curses.wrapper(main_loop)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nDashboard closed. Drone continues flying.")


if __name__ == '__main__':
    main()
