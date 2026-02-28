#!/usr/bin/env python3
"""
UAV Paper Data Analysis Script
================================
Reads all .jsonl flight files + terminal logs + Pi hardware logs
and outputs formatted tables ready for Overleaf paper.

Usage:
    python3 analyze_paper_data.py

    # Analyze specific config only:
    python3 analyze_paper_data.py --config edge     # Pi edge (1B)
    python3 analyze_paper_data.py --config local    # Local laptop (8B)

    # Analyze specific folder:
    python3 analyze_paper_data.py --dir ~/uav_dataset/

Output:
    - Prints all tables to terminal
    - Saves results/paper_data_summary.txt  (plain text)
    - Saves results/paper_data_summary.csv  (spreadsheet)
    - Saves results/latex_tables.tex        (paste into Overleaf)
"""

import json
import glob
import os
import re
import sys
import argparse
import csv
from collections import Counter, defaultdict
from datetime import datetime

# ── Config ────────────────────────────────────────────────────────────────────
DATASET_DIR   = os.path.expanduser("~/uav_dataset/")
RESULTS_DIR   = os.path.join(DATASET_DIR, "results/")
os.makedirs(RESULTS_DIR, exist_ok=True)

INTENT_NAMES = {
    1: "HOLD_SAFE",
    2: "CONTINUE_ASSIGNED_TASK",
    3: "SEARCH_NEXT_PRIORITY",
    4: "INSPECT_TARGET_REPORT",
    5: "COMMS_REACQUIRE",
    6: "RETURN_TO_BASE",
    7: "ABORT_AND_SAFE",
}

# ── Helpers ───────────────────────────────────────────────────────────────────

def safe_avg(lst):
    return round(sum(lst) / len(lst), 3) if lst else 0.0

def safe_min(lst):
    return round(min(lst), 3) if lst else 0.0

def safe_max(lst):
    return round(max(lst), 3) if lst else 0.0

def load_jsonl(filepath):
    records = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return records

def detect_config(filepath, records):
    """Detect if this file is edge (Pi 1B) or local (laptop 8B)."""
    # Method 1: edge_metrics key only exists in Pi edge files
    if any("edge_metrics" in r for r in records):
        return "edge_1b"
    # Method 2: filename hint
    name = os.path.basename(filepath).lower()
    if "edge" in name or "pi" in name or "1b" in name:
        return "edge_1b"
    if "local" in name or "8b" in name or "laptop" in name:
        return "local_8b"
    # Method 3: response time — local llama3:8b ~2-8s, Pi edge ~15-100s
    times = [r["llm_output"]["response_time_seconds"] for r in records
             if not r["llm_output"]["fallback_used"]]
    avg_t = sum(times) / len(times) if times else 0
    if avg_t > 10.0:
        return "edge_1b"
    return "local_8b"

def load_terminal_log(filepath):
    """Load a terminal log file and extract key events."""
    try:
        with open(filepath) as f:
            return f.read()
    except:
        return ""

def count_pattern(text, pattern):
    return len(re.findall(pattern, text, re.IGNORECASE))

# ── Load all data ─────────────────────────────────────────────────────────────

def load_all_data(dataset_dir):
    jsonl_files  = glob.glob(os.path.join(dataset_dir, "flight_*.jsonl"))
    terminal_logs = glob.glob(os.path.join(dataset_dir, "run_*_terminal.log"))
    pi_logs      = glob.glob(os.path.join(dataset_dir, "pi_hardware_log_*.txt"))

    configs = defaultdict(lambda: {
        "records": [],
        "terminal_logs": [],
        "pi_logs": [],
        "files": [],
    })

    for f in sorted(jsonl_files):
        records = load_jsonl(f)
        if not records:
            continue
        cfg = detect_config(f, records)
        configs[cfg]["records"].extend(records)
        configs[cfg]["files"].append(os.path.basename(f))

    for f in terminal_logs:
        text = load_terminal_log(f)
        # Try to detect which config this log belongs to
        if "llama3.2:1b" in text or "edge" in f.lower():
            configs["edge_1b"]["terminal_logs"].append(text)
        elif "llama3" in text or "8b" in f.lower():
            configs["local_8b"]["terminal_logs"].append(text)
        else:
            # Add to both
            configs["edge_1b"]["terminal_logs"].append(text)
            configs["local_8b"]["terminal_logs"].append(text)

    for f in pi_logs:
        text = load_terminal_log(f)
        configs["edge_1b"]["pi_logs"].append(text)

    return configs

# ── Section A: Core LLM Performance ──────────────────────────────────────────

def analyze_section_a(configs):
    print("\n" + "="*70)
    print("  SECTION A — CORE LLM PERFORMANCE")
    print("="*70)

    rows = []
    for cfg_name, data in configs.items():
        records = data["records"]
        if not records:
            continue

        latencies  = [r["llm_output"]["response_time_seconds"] for r in records
                      if not r["llm_output"]["fallback_used"]]
        confs      = [r["llm_output"]["confidence"] for r in records
                      if not r["llm_output"]["fallback_used"]]
        fallbacks  = sum(1 for r in records if r["llm_output"]["fallback_used"])
        total      = len(records)

        # Count JSON parse failures from terminal logs
        json_fails = 0
        for log in data["terminal_logs"]:
            json_fails += count_pattern(log, r"No JSON in response")
            json_fails += count_pattern(log, r"LLM.*Failed.*JSON")

        # Count timeouts
        timeouts = 0
        for log in data["terminal_logs"]:
            timeouts += count_pattern(log, r"timed out")
        # Also check fallback reasoning
        timeouts += sum(1 for r in records
                        if "timed out" in r["llm_output"].get("reasoning", "").lower())

        label = "Pi Edge (llama3.2:1b)" if cfg_name == "edge_1b" else \
                "Local (llama3:8b)"     if cfg_name == "local_8b" else cfg_name

        row = {
            "Config":           label,
            "Total Decisions":  total,
            "Avg Latency (s)":  safe_avg(latencies),
            "Min Latency (s)":  safe_min(latencies),
            "Max Latency (s)":  safe_max(latencies),
            "Avg Confidence %": safe_avg(confs),
            "Fallbacks":        fallbacks,
            "JSON Failures":    json_fails,
            "Timeouts":         timeouts,
        }
        rows.append(row)

        print(f"\n  Config          : {label}")
        print(f"  Flight files    : {', '.join(data['files']) or 'none'}")
        print(f"  Total decisions : {total}")
        print(f"  Avg latency     : {row['Avg Latency (s)']:.2f}s")
        print(f"  Min latency     : {row['Min Latency (s)']:.2f}s")
        print(f"  Max latency     : {row['Max Latency (s)']:.2f}s")
        print(f"  Avg confidence  : {row['Avg Confidence %']:.1f}%")
        print(f"  Fallbacks       : {fallbacks}")
        print(f"  JSON failures   : {json_fails}")
        print(f"  Timeouts        : {timeouts}")

    return rows

# ── Section B: Intent Decision Quality ───────────────────────────────────────

def analyze_section_b(configs):
    print("\n" + "="*70)
    print("  SECTION B — INTENT DECISION QUALITY")
    print("="*70)

    all_intent_rows = []
    unsafe_examples = []

    for cfg_name, data in configs.items():
        records = data["records"]
        if not records:
            continue

        label = "Pi Edge (1B)" if cfg_name == "edge_1b" else \
                "Local (8B)"   if cfg_name == "local_8b" else cfg_name

        intent_counts = Counter(r["llm_output"]["intent_name"] for r in records)
        print(f"\n  [{label}] Intent Distribution:")
        for intent in INTENT_NAMES.values():
            count = intent_counts.get(intent, 0)
            pct   = count / len(records) * 100 if records else 0
            bar   = "█" * int(pct / 5)
            print(f"    {intent:<35} : {count:3d} ({pct:5.1f}%) {bar}")
            all_intent_rows.append({
                "Config": label,
                "Intent": intent,
                "Count":  count,
                "Pct":    round(pct, 1),
            })

        # Flag potentially unsafe decisions
        for r in records:
            batt  = r["input"]["battery_pct"]
            ekf   = r["input"]["ekf_confidence"]
            intent = r["llm_output"]["intent_name"]
            conf  = r["llm_output"]["confidence"]
            # Unsafe: flying far when battery critical
            if batt < 15 and intent in ["SEARCH_NEXT_PRIORITY", "INSPECT_TARGET_REPORT",
                                         "COMMS_REACQUIRE"]:
                unsafe_examples.append({
                    "Config":   label,
                    "Intent":   intent,
                    "Battery":  batt,
                    "EKF":      ekf,
                    "Conf":     conf,
                    "Reasoning": r["llm_output"]["reasoning"][:120],
                    "Issue":    "Flying far with critical battery",
                })
            # Unsafe: flying far when EKF critical
            if ekf < 0.4 and intent in ["SEARCH_NEXT_PRIORITY", "INSPECT_TARGET_REPORT"]:
                unsafe_examples.append({
                    "Config":   label,
                    "Intent":   intent,
                    "Battery":  batt,
                    "EKF":      ekf,
                    "Conf":     conf,
                    "Reasoning": r["llm_output"]["reasoning"][:120],
                    "Issue":    "Flying far with poor EKF",
                })

    if unsafe_examples:
        print(f"\n  ⚠️  POTENTIALLY UNSAFE DECISIONS ({len(unsafe_examples)} found):")
        for u in unsafe_examples:
            print(f"    [{u['Config']}] {u['Intent']} | Batt={u['Battery']}% EKF={u['EKF']} Conf={u['Conf']}%")
            print(f"    Issue: {u['Issue']}")
            print(f"    Reasoning: {u['Reasoning']}")
    else:
        print("\n  ✅ No unsafe intent decisions detected.")

    return all_intent_rows, unsafe_examples

# ── Section C: Signal Loss Scenarios ─────────────────────────────────────────

def analyze_section_c(configs):
    print("\n" + "="*70)
    print("  SECTION C — SIGNAL LOSS SCENARIOS")
    print("="*70)

    rows = []
    for cfg_name, data in configs.items():
        records = data["records"]
        if not records:
            continue

        label = "Pi Edge (1B)" if cfg_name == "edge_1b" else \
                "Local (8B)"   if cfg_name == "local_8b" else cfg_name

        reentries = [r for r in records if r["llm_output"]["was_reentry_decision"]]
        durations = [r["input"]["last_loss_duration_s"] for r in reentries]
        total_losses = max((r["input"]["signal_loss_count"] for r in records), default=0)

        correct = 0
        incorrect = 0
        reentry_details = []
        for r in reentries:
            batt     = r["input"]["battery_pct"]
            chosen   = r["llm_output"]["intent_name"]
            pre_loss = r["input"]["pre_loss_intent"]
            duration = r["input"]["last_loss_duration_s"]
            # Expected logic:
            if batt < 15:
                expected = "RETURN_TO_BASE"
            elif batt < 25:
                expected = "RETURN_TO_BASE"
            else:
                expected = pre_loss if pre_loss != "None" else "SEARCH_NEXT_PRIORITY"

            match = chosen == expected or \
                    (expected == "SEARCH_NEXT_PRIORITY" and
                     chosen in ["SEARCH_NEXT_PRIORITY", "CONTINUE_ASSIGNED_TASK"])
            if match:
                correct += 1
            else:
                incorrect += 1

            reentry_details.append({
                "Config":    label,
                "Duration":  duration,
                "Battery":   batt,
                "Pre-loss":  pre_loss,
                "Chosen":    chosen,
                "Expected":  expected,
                "Match":     "✅" if match else "❌",
            })

        print(f"\n  [{label}]")
        print(f"  Total signal losses recorded : {total_losses}")
        print(f"  Re-entry decisions made      : {len(reentries)}")
        print(f"  Avg loss duration            : {safe_avg(durations):.1f}s")
        print(f"  Min/Max loss duration        : {safe_min(durations):.1f}s / {safe_max(durations):.1f}s")
        print(f"  Correct re-entry intent      : {correct}/{len(reentries)}")
        print(f"  Incorrect re-entry intent    : {incorrect}/{len(reentries)}")
        if reentry_details:
            print(f"\n  Re-entry decision breakdown:")
            for d in reentry_details:
                print(f"    {d['Match']} Batt={d['Battery']}% | Loss={d['Duration']:.1f}s | "
                      f"Was={d['Pre-loss']} → Chose={d['Chosen']} (expected≈{d['Expected']})")

        rows.append({
            "Config":           label,
            "Total Losses":     total_losses,
            "Reentry Decisions": len(reentries),
            "Avg Duration (s)": safe_avg(durations),
            "Correct Reentry":  correct,
            "Incorrect Reentry": incorrect,
            "Accuracy %":       round(correct/len(reentries)*100, 1) if reentries else 0,
        })

    return rows

# ── Section D: Safety Gate ────────────────────────────────────────────────────

def analyze_section_d(configs):
    print("\n" + "="*70)
    print("  SECTION D — SAFETY GATE ANALYSIS")
    print("="*70)

    rows = []
    for cfg_name, data in configs.items():
        records = data["records"]
        label = "Pi Edge (1B)" if cfg_name == "edge_1b" else \
                "Local (8B)"   if cfg_name == "local_8b" else cfg_name

        # Count from terminal logs
        ekf_gates  = 0
        batt_gates = 0
        for log in data["terminal_logs"]:
            ekf_gates  += count_pattern(log, r"Safety gate.*EKF critical")
            ekf_gates  += count_pattern(log, r"EKF critical.*skipping LLM")
            batt_gates += count_pattern(log, r"Safety gate.*battery")
            batt_gates += count_pattern(log, r"Battery CRITICAL.*skipping LLM")

        # Also check dataset — safety gate decisions have fallback=True
        # and reasoning containing "safety gate"
        gate_records = [r for r in records
                        if "safety gate" in r["llm_output"].get("reasoning","").lower()]
        ekf_gate_records  = [r for r in gate_records
                              if "ekf" in r["llm_output"].get("reasoning","").lower()]
        batt_gate_records = [r for r in gate_records
                              if "battery" in r["llm_output"].get("reasoning","").lower()]

        total_gates = ekf_gates + batt_gates + len(gate_records)
        always_correct = True  # We verify below

        # Verify correctness: safety gate should always produce HOLD_SAFE or RTB
        for r in gate_records:
            intent = r["llm_output"]["intent_name"]
            if intent not in ["HOLD_SAFE", "RETURN_TO_BASE", "ABORT_AND_SAFE"]:
                always_correct = False

        print(f"\n  [{label}]")
        print(f"  EKF safety gate fires   : {ekf_gates + len(ekf_gate_records)}")
        print(f"  Batt safety gate fires  : {batt_gates + len(batt_gate_records)}")
        print(f"  Total safety gate fires : {total_gates}")
        print(f"  Always fired correctly  : {'✅ YES' if always_correct else '❌ NO — check records'}")

        rows.append({
            "Config":          label,
            "EKF Gate Fires":  ekf_gates + len(ekf_gate_records),
            "Batt Gate Fires": batt_gates + len(batt_gate_records),
            "Total Fires":     total_gates,
            "Always Correct":  "YES" if always_correct else "NO",
        })

    return rows

# ── Section E: Pi Hardware ────────────────────────────────────────────────────

def analyze_section_e(configs):
    print("\n" + "="*70)
    print("  SECTION E — RASPBERRY PI HARDWARE METRICS")
    print("="*70)

    data = configs.get("edge_1b", {})
    pi_logs = data.get("pi_logs", [])
    records = data.get("records", [])

    # Parse hardware logs
    cpu_values  = []
    ram_used    = []
    ram_total   = []
    temps       = []

    for log in pi_logs:
        for line in log.splitlines():
            # CPU
            cpu_match = re.search(r'CPU:\s*([\d.]+)%', line)
            if cpu_match:
                cpu_values.append(float(cpu_match.group(1)))
            # RAM: "263/15800 MB"
            ram_match = re.search(r'RAM:\s*(\d+)/(\d+)\s*MB', line)
            if ram_match:
                ram_used.append(int(ram_match.group(1)))
                ram_total.append(int(ram_match.group(2)))
            # Temp
            temp_match = re.search(r'temp=([\d.]+)', line)
            if temp_match:
                temps.append(float(temp_match.group(1)))

    # Edge metrics from jsonl
    edge_inferences = []
    for r in records:
        em = r.get("edge_metrics", {})
        if em:
            edge_inferences.append(em)

    inf_times  = [e.get("inference_time_s", 0) for e in edge_inferences if e.get("inference_time_s")]
    tok_per_s  = [e.get("tokens_per_sec", 0)   for e in edge_inferences if e.get("tokens_per_sec")]
    rtts       = [e.get("network_rtt_s", 0)     for e in edge_inferences if e.get("network_rtt_s")]
    out_tokens = [e.get("output_tokens", 0)     for e in edge_inferences if e.get("output_tokens")]
    in_tokens  = [e.get("prompt_tokens", 0)     for e in edge_inferences if e.get("prompt_tokens")]

    timeouts = sum(1 for r in records
                   if "timed out" in r["llm_output"].get("reasoning","").lower())
    for log in data.get("terminal_logs", []):
        timeouts += count_pattern(log, r"timed out")

    print(f"\n  Hardware logs found    : {len(pi_logs)}")
    print(f"  Edge metric records    : {len(edge_inferences)}")
    print()
    if cpu_values:
        print(f"  CPU Usage:")
        print(f"    Average            : {safe_avg(cpu_values):.1f}%")
        print(f"    Peak               : {safe_max(cpu_values):.1f}%")
        print(f"    Idle (min)         : {safe_min(cpu_values):.1f}%")
    if ram_used:
        print(f"  RAM Usage:")
        print(f"    Average used       : {safe_avg(ram_used):.0f} MB")
        print(f"    Peak used          : {safe_max(ram_used):.0f} MB")
        print(f"    Total available    : {safe_avg(ram_total):.0f} MB")
    if temps:
        print(f"  Temperature:")
        print(f"    Average            : {safe_avg(temps):.1f}°C")
        print(f"    Peak               : {safe_max(temps):.1f}°C")
    if inf_times:
        print(f"  Inference Performance:")
        print(f"    Avg inference time : {safe_avg(inf_times):.2f}s")
        print(f"    Min inference time : {safe_min(inf_times):.2f}s")
        print(f"    Max inference time : {safe_max(inf_times):.2f}s")
        print(f"    Avg tokens/sec     : {safe_avg(tok_per_s):.1f}")
        print(f"    Avg network RTT    : {safe_avg(rtts):.2f}s")
        print(f"    Avg prompt tokens  : {safe_avg(in_tokens):.0f}")
        print(f"    Avg output tokens  : {safe_avg(out_tokens):.0f}")
    print(f"  Timeouts               : {timeouts}")

    return {
        "avg_cpu":        safe_avg(cpu_values),
        "peak_cpu":       safe_max(cpu_values),
        "avg_ram_mb":     safe_avg(ram_used),
        "peak_ram_mb":    safe_max(ram_used),
        "avg_temp_c":     safe_avg(temps),
        "peak_temp_c":    safe_max(temps),
        "avg_inference_s": safe_avg(inf_times),
        "min_inference_s": safe_min(inf_times),
        "max_inference_s": safe_max(inf_times),
        "avg_tok_per_s":  safe_avg(tok_per_s),
        "avg_rtt_s":      safe_avg(rtts),
        "timeouts":       timeouts,
    }

# ── LaTeX Table Generator ─────────────────────────────────────────────────────

def generate_latex(sec_a, sec_c, sec_d, sec_e):
    lines = []
    lines.append("% ============================================================")
    lines.append("% AUTO-GENERATED LaTeX Tables — paste into Overleaf")
    lines.append(f"% Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("% ============================================================\n")

    # Table: Section A
    lines.append("% TABLE: Core LLM Performance (Section A)")
    lines.append("\\begin{table}[h]")
    lines.append("\\centering")
    lines.append("\\caption{LLM Inference Performance Comparison}")
    lines.append("\\label{tab:llm_performance}")
    lines.append("\\begin{tabular}{lrrrrrrr}")
    lines.append("\\hline")
    lines.append("\\textbf{Config} & \\textbf{Decisions} & \\textbf{Avg Lat (s)} & "
                 "\\textbf{Min (s)} & \\textbf{Max (s)} & \\textbf{Avg Conf} & "
                 "\\textbf{Fallbacks} & \\textbf{Timeouts} \\\\")
    lines.append("\\hline")
    for row in sec_a:
        lines.append(
            f"{row['Config']} & {row['Total Decisions']} & "
            f"{row['Avg Latency (s)']:.2f} & {row['Min Latency (s)']:.2f} & "
            f"{row['Max Latency (s)']:.2f} & {row['Avg Confidence %']:.1f}\\% & "
            f"{row['Fallbacks']} & {row['Timeouts']} \\\\"
        )
    lines.append("\\hline")
    lines.append("\\end{tabular}")
    lines.append("\\end{table}\n")

    # Table: Section C
    lines.append("% TABLE: Signal Loss Recovery (Section C)")
    lines.append("\\begin{table}[h]")
    lines.append("\\centering")
    lines.append("\\caption{Signal Loss Recovery Analysis}")
    lines.append("\\label{tab:signal_loss}")
    lines.append("\\begin{tabular}{lrrrrrr}")
    lines.append("\\hline")
    lines.append("\\textbf{Config} & \\textbf{Losses} & \\textbf{Reentries} & "
                 "\\textbf{Avg Dur (s)} & \\textbf{Correct} & "
                 "\\textbf{Incorrect} & \\textbf{Accuracy} \\\\")
    lines.append("\\hline")
    for row in sec_c:
        lines.append(
            f"{row['Config']} & {row['Total Losses']} & {row['Reentry Decisions']} & "
            f"{row['Avg Duration (s)']:.1f} & {row['Correct Reentry']} & "
            f"{row['Incorrect Reentry']} & {row['Accuracy %']:.1f}\\% \\\\"
        )
    lines.append("\\hline")
    lines.append("\\end{tabular}")
    lines.append("\\end{table}\n")

    # Table: Section D
    lines.append("% TABLE: Safety Gate (Section D)")
    lines.append("\\begin{table}[h]")
    lines.append("\\centering")
    lines.append("\\caption{Safety Gate Activation Analysis}")
    lines.append("\\label{tab:safety_gate}")
    lines.append("\\begin{tabular}{lrrrr}")
    lines.append("\\hline")
    lines.append("\\textbf{Config} & \\textbf{EKF Gates} & \\textbf{Batt Gates} & "
                 "\\textbf{Total} & \\textbf{Always Correct} \\\\")
    lines.append("\\hline")
    for row in sec_d:
        lines.append(
            f"{row['Config']} & {row['EKF Gate Fires']} & {row['Batt Gate Fires']} & "
            f"{row['Total Fires']} & {row['Always Correct']} \\\\"
        )
    lines.append("\\hline")
    lines.append("\\end{tabular}")
    lines.append("\\end{table}\n")

    # Table: Section E
    lines.append("% TABLE: Pi 5 Hardware Metrics (Section E)")
    lines.append("\\begin{table}[h]")
    lines.append("\\centering")
    lines.append("\\caption{Raspberry Pi 5 Edge Inference Hardware Metrics}")
    lines.append("\\label{tab:pi_hardware}")
    lines.append("\\begin{tabular}{lr}")
    lines.append("\\hline")
    lines.append("\\textbf{Metric} & \\textbf{Value} \\\\")
    lines.append("\\hline")
    metrics = [
        ("Avg CPU Usage",          f"{sec_e['avg_cpu']:.1f}\\%"),
        ("Peak CPU Usage",         f"{sec_e['peak_cpu']:.1f}\\%"),
        ("Avg RAM Used",           f"{sec_e['avg_ram_mb']:.0f} MB"),
        ("Peak RAM Used",          f"{sec_e['peak_ram_mb']:.0f} MB"),
        ("Avg Temperature",        f"{sec_e['avg_temp_c']:.1f}°C"),
        ("Peak Temperature",       f"{sec_e['peak_temp_c']:.1f}°C"),
        ("Avg Inference Time",     f"{sec_e['avg_inference_s']:.2f}s"),
        ("Min Inference Time",     f"{sec_e['min_inference_s']:.2f}s"),
        ("Max Inference Time",     f"{sec_e['max_inference_s']:.2f}s"),
        ("Avg Throughput",         f"{sec_e['avg_tok_per_s']:.1f} tok/s"),
        ("Avg Network RTT",        f"{sec_e['avg_rtt_s']:.2f}s"),
        ("Inference Timeouts",     f"{sec_e['timeouts']}"),
    ]
    for k, v in metrics:
        if v.startswith("0.0") or v == "0%" or v == "0.00s":
            v = v + " \\textsuperscript{*}"
        lines.append(f"{k} & {v} \\\\")
    lines.append("\\hline")
    lines.append("\\multicolumn{2}{l}{\\textsuperscript{*}No hardware logs found — run Pi monitor script} \\\\")
    lines.append("\\end{tabular}")
    lines.append("\\end{table}\n")

    lines.append("% NOTE: Run Pi monitor during flight:")
    lines.append("% ssh niche@10.0.0.119")
    lines.append("% while true; do echo \"$(date) | CPU: $(top -bn1 | grep 'Cpu(s)' | awk '{print $2}')% | RAM: $(free -m | awk 'NR==2{printf \"%s/%s MB\", $3,$2}') | TEMP: $(vcgencmd measure_temp)\"; sleep 5; done | tee ~/pi_hardware_log_$(date +%Y%m%d_%H%M%S).txt")

    return "\n".join(lines)

# ── Save Results ──────────────────────────────────────────────────────────────

def save_csv(sec_a, sec_b_intents, sec_c, sec_d, results_dir):
    # Section A CSV
    path = os.path.join(results_dir, "section_a_llm_performance.csv")
    with open(path, 'w', newline='') as f:
        if sec_a:
            writer = csv.DictWriter(f, fieldnames=sec_a[0].keys())
            writer.writeheader()
            writer.writerows(sec_a)
    print(f"\n  Saved: {path}")

    # Section B CSV
    path = os.path.join(results_dir, "section_b_intent_distribution.csv")
    with open(path, 'w', newline='') as f:
        if sec_b_intents:
            writer = csv.DictWriter(f, fieldnames=sec_b_intents[0].keys())
            writer.writeheader()
            writer.writerows(sec_b_intents)
    print(f"  Saved: {path}")

    # Section C CSV
    path = os.path.join(results_dir, "section_c_signal_loss.csv")
    with open(path, 'w', newline='') as f:
        if sec_c:
            writer = csv.DictWriter(f, fieldnames=sec_c[0].keys())
            writer.writeheader()
            writer.writerows(sec_c)
    print(f"  Saved: {path}")

    # Section D CSV
    path = os.path.join(results_dir, "section_d_safety_gate.csv")
    with open(path, 'w', newline='') as f:
        if sec_d:
            writer = csv.DictWriter(f, fieldnames=sec_d[0].keys())
            writer.writeheader()
            writer.writerows(sec_d)
    print(f"  Saved: {path}")

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="UAV Paper Data Analysis")
    parser.add_argument("--dir", default=DATASET_DIR, help="Dataset directory")
    parser.add_argument("--config", choices=["edge", "local", "all"], default="all")
    args = parser.parse_args()

    print("\n" + "="*70)
    print("  UAV PAPER DATA ANALYSIS SCRIPT")
    print(f"  Dataset dir : {args.dir}")
    print(f"  Config      : {args.config}")
    print(f"  Generated   : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*70)

    configs = load_all_data(args.dir)

    if not any(c["records"] for c in configs.values()):
        print("\n  ❌ No flight data found!")
        print(f"  Looking in: {args.dir}")
        print("  Make sure you have .jsonl files from your flights.")
        print("  Run the UAV system first to collect data.\n")
        sys.exit(1)

    # Filter config if requested
    if args.config == "edge":
        configs = {k: v for k, v in configs.items() if k == "edge_1b"}
    elif args.config == "local":
        configs = {k: v for k, v in configs.items() if k == "local_8b"}

    print(f"\n  Configs found: {list(configs.keys())}")
    for cfg, data in configs.items():
        print(f"    {cfg}: {len(data['records'])} decisions from {len(data['files'])} files")

    # Run all sections
    sec_a               = analyze_section_a(configs)
    sec_b_intents, _    = analyze_section_b(configs)
    sec_c               = analyze_section_c(configs)
    sec_d               = analyze_section_d(configs)
    sec_e               = analyze_section_e(configs)

    # Save outputs
    print("\n" + "="*70)
    print("  SAVING OUTPUTS")
    print("="*70)

    save_csv(sec_a, sec_b_intents, sec_c, sec_d, RESULTS_DIR)

    # LaTeX tables
    latex = generate_latex(sec_a, sec_c, sec_d, sec_e)
    latex_path = os.path.join(RESULTS_DIR, "latex_tables.tex")
    with open(latex_path, 'w') as f:
        f.write(latex)
    print(f"  Saved: {latex_path}")

    print("\n" + "="*70)
    print("  DONE — Files saved to:", RESULTS_DIR)
    print("  LaTeX tables ready to paste into Overleaf!")
    print("="*70 + "\n")

if __name__ == "__main__":
    main()
