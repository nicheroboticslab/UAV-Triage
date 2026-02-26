#!/usr/bin/env python3
"""
UAV Dataset Labeling Tool — Stage 5
=====================================
Run this after a flight session to label LLM decisions as good or bad.
Good labels become positive examples. Bad labels become corrected training examples.
Together they form your fine-tuning dataset.

Usage:
    python3 uav_label_dataset.py
    python3 uav_label_dataset.py --file ~/uav_dataset/flight_20260221_143022.jsonl
    python3 uav_label_dataset.py --stats   (show stats without labeling)
    python3 uav_label_dataset.py --export  (export labeled data as fine-tuning format)

Controls during labeling:
    Y     = Correct decision
    N     = Wrong decision (prompts for correct intent + reasoning)
    S     = Skip (label later)
    Q     = Quit and save progress
    ENTER = Show full details of current decision
"""

import json
import os
import sys
import glob
import argparse
from datetime import datetime

DATASET_DIR    = os.path.expanduser("~/uav_dataset")
EXPORT_DIR     = os.path.expanduser("~/uav_dataset/export")

INTENT_NAMES = {
    1: "HOLD_SAFE",
    2: "CONTINUE_ASSIGNED_TASK",
    3: "SEARCH_NEXT_PRIORITY",
    4: "INSPECT_TARGET_REPORT",
    5: "COMMS_REACQUIRE",
    6: "RETURN_TO_BASE",
    7: "ABORT_AND_SAFE",
}


# ============================================================
#  FILE HELPERS
# ============================================================

def load_jsonl(filepath: str) -> list:
    records = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return records


def save_jsonl(filepath: str, records: list):
    tmp = filepath + ".tmp"
    with open(tmp, 'w') as f:
        for record in records:
            f.write(json.dumps(record) + "\n")
    os.replace(tmp, filepath)


def pick_latest_file() -> str:
    files = sorted(glob.glob(os.path.join(DATASET_DIR, "flight_*.jsonl")))
    if not files:
        return None
    return files[-1]


# ============================================================
#  DISPLAY HELPERS
# ============================================================

def divider(char="═", width=65):
    print(char * width)

def header(text: str):
    divider()
    print(f"  {text}")
    divider()

def show_decision_summary(record: dict, index: int, total: int):
    """Show a compact one-screen summary of a decision."""
    inp    = record["input"]
    out    = record["llm_output"]
    label  = record["label"]
    outcome = record["outcome"]

    divider("─")
    labeled_tag = ""
    if label["correct"] is True:    labeled_tag = " ✅ [LABELED CORRECT]"
    elif label["correct"] is False: labeled_tag = " ❌ [LABELED WRONG]"

    print(f"  Decision {index + 1} of {total}  |  {record['flight_time']}  |  ID: {record['id']}{labeled_tag}")
    divider("─")

    # Input situation
    batt = inp["battery_pct"]
    ekf  = inp["ekf_confidence"]
    batt_label = "🔴 CRIT" if batt < inp["battery_crit_thresh"] else ("🟡 LOW" if batt < inp["battery_low_thresh"] else "🟢 OK")
    ekf_label  = "🔴 POOR" if ekf < inp["ekf_conf_min"] else ("🟡 MOD" if ekf < 0.5 else "🟢 GOOD")

    print(f"  Situation   : Battery {batt:.1f}% {batt_label}  |  EKF {ekf:.2f} {ekf_label}")
    print(f"  Was heading : {inp['current_wp']}")
    print(f"  Signal losses: {inp['signal_loss_count']}  |  Re-entry: {inp['post_signal_loss_reentry']}")
    if inp["post_signal_loss_reentry"]:
        print(f"  Last loss   : {inp['last_loss_duration_s']:.1f}s  |  Cost: {inp['last_loss_battery_cost']:.1f}%  |  Drift: {inp['last_loss_drift_m']:.1f}m")
    print(f"  Visited     : {inp['sectors_visited'] if inp['sectors_visited'] else 'None'}")
    print(f"  Remaining   : {inp['sectors_remaining']}")

    divider("─")

    # LLM output
    conf       = out["confidence"]
    conf_label = "✅ HIGH" if conf >= 70 else ("⚠️  MOD" if conf >= 50 else "❌ LOW")
    fallback   = " [FALLBACK USED]" if out["fallback_used"] else ""
    reentry    = " [RE-ENTRY]" if out["was_reentry_decision"] else ""

    print(f"  LLM decided : [{out['intent']}] {out['intent_name']} → {out['target_wp']}{fallback}{reentry}")
    print(f"  Confidence  : {conf}%  {conf_label}  |  Response time: {out['response_time_seconds']:.1f}s")
    print()
    print("  Reasoning:")
    # Word wrap reasoning
    words = out["reasoning"].split()
    line = "    "
    for word in words:
        if len(line) + len(word) + 1 <= 66:
            line += ("" if line == "    " else " ") + word
        else:
            print(line)
            line = "    " + word
    if line.strip():
        print(line)
    print()
    print(f"  Checkpoint  : {out['next_checkpoint']}")

    # Outcome if available
    if outcome["waypoint_reached"] is not None:
        reached = "✅ YES" if outcome["waypoint_reached"] else "❌ NO"
        dwell   = " + dwell ✅" if outcome["dwell_completed"] else ""
        print(f"  Outcome     : Reached={reached}{dwell}  |  Time: {outcome['time_to_arrive_seconds']:.0f}s  |  Batt on arrival: {outcome['battery_at_arrival']:.1f}%")
        if outcome["another_signal_loss_occurred"]:
            print("  ⚠️  Another signal loss occurred during this leg")
    else:
        print("  Outcome     : Not yet recorded")

    # Existing label
    if label["correct"] is False:
        print()
        print(f"  ❌ Correct intent: [{label['correct_intent']}] {INTENT_NAMES.get(label['correct_intent'], '?')}")
        if label["correct_reasoning"]:
            print(f"  Better reasoning: {label['correct_reasoning'][:80]}")

    divider("─")


def show_full_details(record: dict):
    """Show complete record including event log."""
    print()
    print(json.dumps(record, indent=2))
    print()


# ============================================================
#  LABELING FLOW
# ============================================================

def label_records(records: list, filepath: str):
    """
    Walk through unlabeled records and prompt for labels.
    Saves progress after each label.
    """
    unlabeled = [i for i, r in enumerate(records) if r["label"]["correct"] is None]

    if not unlabeled:
        print("\n✅ All decisions in this file are already labeled!")
        show_stats(records)
        return

    print(f"\n📋 {len(unlabeled)} unlabeled decisions to review.")
    print("Controls: Y=correct  N=wrong  S=skip  Q=quit  ENTER=full details\n")

    labeled_this_session = 0

    for idx in unlabeled:
        record = records[idx]
        show_decision_summary(record, idx, len(records))

        while True:
            try:
                choice = input("  Label [Y/N/S/Q/ENTER]: ").strip().upper()
            except (EOFError, KeyboardInterrupt):
                print("\n\nSaving and exiting...")
                save_jsonl(filepath, records)
                return

            if choice == "":
                # Show full details and re-prompt
                show_full_details(record)
                show_decision_summary(record, idx, len(records))
                continue

            elif choice == "Y":
                record["label"]["correct"]    = True
                record["label"]["labeled_by"] = "human"
                record["label"]["labeled_at"] = datetime.now().isoformat()
                print("  ✅ Marked as CORRECT\n")
                labeled_this_session += 1
                break

            elif choice == "N":
                print()
                print("  What should the correct intent have been?")
                for k, v in INTENT_NAMES.items():
                    print(f"    {k} = {v}")
                print()

                while True:
                    try:
                        intent_input = input("  Correct intent [1-7]: ").strip()
                    except (EOFError, KeyboardInterrupt):
                        intent_input = ""
                    if intent_input.isdigit() and int(intent_input) in range(1, 8):
                        correct_intent = int(intent_input)
                        break
                    print("  Please enter a number 1-7.")

                print()
                print("  Write better reasoning (or press ENTER to leave blank):")
                print("  (Explain why your chosen intent was better)")
                try:
                    correct_reasoning = input("  > ").strip()
                except (EOFError, KeyboardInterrupt):
                    correct_reasoning = ""

                print()
                try:
                    notes = input("  Any additional notes? (ENTER to skip): ").strip()
                except (EOFError, KeyboardInterrupt):
                    notes = ""

                record["label"]["correct"]          = False
                record["label"]["correct_intent"]   = correct_intent
                record["label"]["correct_reasoning"] = correct_reasoning if correct_reasoning else None
                record["label"]["notes"]            = notes if notes else None
                record["label"]["labeled_by"]       = "human"
                record["label"]["labeled_at"]       = datetime.now().isoformat()

                print(f"\n  ❌ Marked as WRONG → correct: [{correct_intent}] {INTENT_NAMES[correct_intent]}\n")
                labeled_this_session += 1
                break

            elif choice == "S":
                print("  ⏭  Skipped.\n")
                break

            elif choice == "Q":
                save_jsonl(filepath, records)
                print(f"\n💾 Saved. Labeled {labeled_this_session} decisions this session.")
                show_stats(records)
                return

            else:
                print("  Please enter Y, N, S, Q, or ENTER.")

        # Save after every label
        save_jsonl(filepath, records)

    print(f"\n✅ Done! Labeled {labeled_this_session} decisions this session.")
    show_stats(records)


# ============================================================
#  STATS
# ============================================================

def show_stats(records: list):
    total     = len(records)
    labeled   = sum(1 for r in records if r["label"]["correct"] is not None)
    correct   = sum(1 for r in records if r["label"]["correct"] is True)
    wrong     = sum(1 for r in records if r["label"]["correct"] is False)
    unlabeled = total - labeled
    fallbacks = sum(1 for r in records if r["llm_output"]["fallback_used"])
    reentries = sum(1 for r in records if r["llm_output"]["was_reentry_decision"])
    arrived   = sum(1 for r in records if r["outcome"]["waypoint_reached"] is True)

    # Intent distribution
    intent_counts = {}
    for r in records:
        name = r["llm_output"]["intent_name"]
        intent_counts[name] = intent_counts.get(name, 0) + 1

    divider()
    print("  DATASET STATS")
    divider()
    print(f"  Total decisions  : {total}")
    print(f"  Labeled          : {labeled}  ({correct} correct, {wrong} wrong)")
    print(f"  Unlabeled        : {unlabeled}")
    print(f"  Fallbacks used   : {fallbacks}")
    print(f"  Re-entry decisions: {reentries}")
    print(f"  Waypoints reached: {arrived}")
    print()
    print("  Intent distribution:")
    for intent_name, count in sorted(intent_counts.items(), key=lambda x: -x[1]):
        bar = "█" * count
        print(f"    {intent_name:<30} {bar} ({count})")
    divider()


# ============================================================
#  EXPORT FOR FINE-TUNING
# ============================================================

def export_for_finetuning(records: list, output_path: str):
    """
    Export labeled records in Unsloth/SFT training format.

    Format per example:
    {
      "instruction": "You are an autonomous UAV pilot...",
      "input": "<situation context>",
      "output": "<correct JSON response>"
    }

    Correct decisions → LLM reasoning used as-is
    Wrong decisions   → Correct intent + human-written reasoning used
    """
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    examples = []
    skipped  = 0

    for record in records:
        label = record["label"]
        inp   = record["input"]
        out   = record["llm_output"]

        # Only export labeled records
        if label["correct"] is None:
            skipped += 1
            continue

        # Build situation context string
        batt_status = (
            "GOOD" if inp["battery_pct"] >= inp["battery_low_thresh"] * 2 else
            "MODERATE" if inp["battery_pct"] >= inp["battery_low_thresh"] else
            "LOW — RTB soon" if inp["battery_pct"] >= inp["battery_crit_thresh"] else
            "CRITICAL — land immediately"
        )
        ekf_status = (
            "GOOD" if inp["ekf_confidence"] >= 0.75 else
            "MODERATE" if inp["ekf_confidence"] >= 0.5 else
            "POOR" if inp["ekf_confidence"] >= inp["ekf_conf_min"] else
            "CRITICAL"
        )

        signal_ctx = ""
        if inp["post_signal_loss_reentry"]:
            signal_ctx = (
                f"\nSIGNAL LOSS JUST OCCURRED: "
                f"Duration {inp['last_loss_duration_s']:.1f}s, "
                f"battery cost {inp['last_loss_battery_cost']:.1f}%, "
                f"drift {inp['last_loss_drift_m']:.1f}m. "
                f"Was interrupted: {inp['pre_loss_intent']} → {inp['pre_loss_wp']}."
            )

        situation = (
            f"Battery: {inp['battery_pct']:.1f}% — {batt_status}\n"
            f"EKF: {inp['ekf_confidence']:.2f} — {ekf_status}\n"
            f"Position: {inp['current_pos']}\n"
            f"Was heading to: {inp['current_wp']}\n"
            f"Signal losses this flight: {inp['signal_loss_count']}\n"
            f"Sectors visited: {inp['sectors_visited']}\n"
            f"Sectors remaining: {inp['sectors_remaining']}"
            f"{signal_ctx}\n"
            f"Recent flight log:\n" +
            "\n".join(f"  {e}" for e in inp["event_log_at_decision"][-8:])
        )

        # Determine correct output
        if label["correct"] is True:
            # LLM was right — use its output
            correct_intent    = out["intent"]
            correct_intent_nm = out["intent_name"]
            correct_reasoning = out["reasoning"]
            correct_checkpoint = out["next_checkpoint"]
        else:
            # LLM was wrong — use human correction
            correct_intent    = label["correct_intent"]
            correct_intent_nm = INTENT_NAMES.get(correct_intent, "UNKNOWN")
            correct_reasoning = (
                label["correct_reasoning"] if label["correct_reasoning"]
                else f"The correct action is {correct_intent_nm} given the current conditions."
            )
            correct_checkpoint = out["next_checkpoint"]  # Keep checkpoint as-is

        output_json = json.dumps({
            "intent":          correct_intent,
            "reasoning":       correct_reasoning,
            "confidence":      90 if label["correct"] else 95,
            "next_checkpoint": correct_checkpoint,
        })

        example = {
            "instruction": (
                "You are the autonomous pilot brain of a search-and-rescue UAV. "
                "A human operator is unavailable. "
                "Think step by step and decide the best action. "
                "Return a JSON object with intent (1-7), reasoning, confidence (0-100), "
                "and next_checkpoint."
            ),
            "input":  situation,
            "output": output_json,
        }

        examples.append(example)

    # Write export file
    with open(output_path, 'w') as f:
        for ex in examples:
            f.write(json.dumps(ex) + "\n")

    print(f"\n✅ Exported {len(examples)} training examples to: {output_path}")
    print(f"   Skipped {skipped} unlabeled records.")
    print()
    print("   To fine-tune with Unsloth:")
    print(f"   dataset = load_dataset('json', data_files='{output_path}')")
    print()

    # Also write a summary
    correct_count = sum(1 for r in records if r["label"]["correct"] is True)
    wrong_count   = sum(1 for r in records if r["label"]["correct"] is False)
    print(f"   Breakdown: {correct_count} correct examples + {wrong_count} corrected examples")


# ============================================================
#  MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="UAV Dataset Labeling Tool")
    parser.add_argument("--file",   help="JSONL file to label (default: latest)")
    parser.add_argument("--stats",  action="store_true", help="Show stats only")
    parser.add_argument("--export", action="store_true", help="Export labeled data for fine-tuning")
    parser.add_argument("--all",    action="store_true", help="Label all files in dataset dir")
    args = parser.parse_args()

    os.makedirs(DATASET_DIR, exist_ok=True)

    # Pick file(s) to process
    if args.all:
        files = sorted(glob.glob(os.path.join(DATASET_DIR, "flight_*.jsonl")))
        if not files:
            print(f"No dataset files found in {DATASET_DIR}")
            return
    else:
        filepath = args.file or pick_latest_file()
        if not filepath or not os.path.exists(filepath):
            print(f"No dataset files found in {DATASET_DIR}")
            print("Run a flight session first with autonomous_uav_stage5.py")
            return
        files = [filepath]

    for filepath in files:
        print(f"\n{'='*65}")
        print(f"  FILE: {os.path.basename(filepath)}")
        print(f"{'='*65}")

        records = load_jsonl(filepath)
        if not records:
            print("  Empty file, skipping.")
            continue

        if args.stats:
            show_stats(records)

        elif args.export:
            session_name = os.path.basename(filepath).replace(".jsonl", "")
            output_path  = os.path.join(EXPORT_DIR, f"{session_name}_training.jsonl")
            show_stats(records)
            export_for_finetuning(records, output_path)

        else:
            # Default: label then optionally export
            show_stats(records)
            print()
            label_records(records, filepath)

            # Ask about export after labeling
            try:
                do_export = input("\nExport labeled data for fine-tuning? [Y/N]: ").strip().upper()
            except (EOFError, KeyboardInterrupt):
                do_export = "N"

            if do_export == "Y":
                session_name = os.path.basename(filepath).replace(".jsonl", "")
                output_path  = os.path.join(EXPORT_DIR, f"{session_name}_training.jsonl")
                export_for_finetuning(records, output_path)


if __name__ == "__main__":
    main()
