# 🚁 UAV Autonomous Triage System

> An intelligent UAV system combining **PX4 autopilot**, **ROS 2 Humble**, and a **local LLM (LLaMA3 via Ollama)** for autonomous mission decision-making, battery-aware navigation, signal loss memory, live operator display, and automatic dataset collection for fine-tuning — simulated in **Gazebo** and monitored via **QGroundControl**.

<br>

## 📋 Table of Contents

- [Project Overview](#-project-overview)
- [What's New — 5-Stage Evolution](#-whats-new--5-stage-evolution)
- [System Architecture](#-system-architecture)
- [Prerequisites & Software Stack](#-prerequisites--software-stack)
- [Installation Guide](#-installation-guide)
  - [1. ROS 2 Humble](#1-ros-2-humble)
  - [2. PX4 Autopilot + Gazebo SITL](#2-px4-autopilot--gazebo-sitl)
  - [3. Micro XRCE-DDS Agent](#3-micro-xrce-dds-agent)
  - [4. QGroundControl](#4-qgroundcontrol)
  - [5. Ollama + LLaMA3](#5-ollama--llama3)
  - [6. Python Dependencies](#6-python-dependencies)
  - [7. Build the ROS 2 Workspace](#7-build-the-ros-2-workspace)
- [Running the System](#-running-the-system)
- [Keyboard Controls & Test Keys](#-keyboard-controls--test-keys)
- [How the LLM Brain Works](#-how-the-llm-brain-works)
- [Dataset Collection & Fine-Tuning](#-dataset-collection--fine-tuning)
- [Project Structure](#-project-structure)
- [Troubleshooting](#-troubleshooting)
- [Contributors](#-contributors)

<br>

---

## 🧠 Project Overview

This project implements a **fully autonomous UAV triage architecture** where the drone:

- Flies manually via keyboard in real-time at 20 Hz
- Switches to **autonomous mode** after 15 seconds of no keyboard input
- Uses a **local LLM (LLaMA3)** to think like a real pilot — with flight memory, transparent reasoning, confidence scores, and uncertainty handling
- Visits **all 5 search sectors** in priority order before resetting for a new sweep
- **Remembers signal loss events** and makes smart re-entry decisions after comms restore
- Shows a **live operator dashboard** in a second terminal with full LLM reasoning visible
- **Records every decision** to a dataset file for future fine-tuning
- Hard safety gate bypasses the LLM entirely for critical battery/EKF states

All simulation runs **100% locally** using PX4 SITL + Gazebo with no cloud dependencies.

<br>

---

## 🆕 What's New — 5-Stage Evolution

This project was built incrementally across 5 stages. Each stage added a layer of intelligence on top of the previous one.

### Stage 1 — Event Log System
The LLM previously had zero memory of what happened during a flight. Now every significant event is timestamped and logged, and the full flight history is passed to every LLM query. The LLM can see what it already tried and avoid repeating itself.

```
[T+0s]   System started. Battery: 100%. EKF: 1.00
[T+15s]  Switched to AUTONOMOUS. No input: 15s.
[T+89s]  Arrived at SECTOR_A. Battery: 94%.
[T+110s] LLM (86%): SEARCH_NEXT_PRIORITY → SECTOR_B
```

### Stage 2 — Pilot Brain Prompt
Replaced rigid numbered rules with a genuine reasoning prompt. The LLM now returns structured JSON with `intent`, `reasoning`, `confidence`, and `next_checkpoint`. A confidence gate decides whether to execute, warn, or fall back to deterministic safety logic.

```json
{
  "intent": 3,
  "reasoning": "Battery at 67% is healthy. EKF solid at 0.91. SECTOR_A visited. SECTOR_B is next priority.",
  "confidence": 86,
  "next_checkpoint": "Complete SECTOR_B or battery drops below 25%"
}
```

**Confidence gate:**

| Confidence | Action |
|------------|--------|
| ≥ 70% | Execute decision |
| 50–69% | Execute with warning logged |
| < 50% | Ignore LLM — use deterministic safe fallback |

### Stage 3 — Signal Loss Memory
When signal is lost and regained, the LLM gets a full re-entry context block describing how long the signal was gone, battery cost during loss, position drift, what task was interrupted, and whether this is a repeated pattern. The LLM explicitly addresses this before deciding whether to resume or change plan.

### Stage 4 — Live Operator Dashboard
A standalone `uav_dashboard.py` reads a shared state file written every 0.5s and renders a live terminal display — showing battery bar, EKF status, signal state, full LLM reasoning, confidence level, and the last 8 flight log entries. All updating in real time without scrolling through ROS logs.

### Stage 5 — Dataset Collection & Labeling
Every LLM decision is automatically recorded to `~/uav_dataset/flight_YYYYMMDD_HHMMSS.jsonl` with full input context, LLM output, and outcome tracking (did the drone arrive? battery on arrival? another signal loss?). A labeling tool lets you review decisions after each flight and export a fine-tuning dataset in Unsloth/SFT format.

<br>

---

## 🏗 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         INPUTS                              │
│          Keyboard · Battery · EKF · Position (20Hz)        │
└───────────────────────────┬─────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────┐
│                  COMM LOSS DETECTOR                         │
│    Scores 3 signals → triggers AUTONOMOUS if score ≥ 1     │
└───────────────────────────┬─────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────┐
│                HARD SAFETY GATE                             │
│   Battery < 15% → force RTB  |  EKF < 0.4 → force HOLD    │
│   Bypasses LLM entirely — deterministic and instant         │
└───────────────────────────┬─────────────────────────────────┘
                            │ (only if sensors are safe)
┌───────────────────────────▼─────────────────────────────────┐
│                 LLM DECISION ENGINE                         │
│    Ollama / LLaMA3 8B — thinks like a real pilot            │
│    Input : battery, EKF, flight log, signal loss context   │
│    Output: intent + reasoning + confidence + checkpoint     │
│    Runs in background thread (non-blocking)                 │
│    Confidence gate: ≥70 execute | 50-69 warn | <50 fallback │
└───────────────────────────┬─────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────┐
│                WAYPOINT GENERATOR                           │
│    Converts intent → battery-aware [x, y, z]               │
│    Tracks all visited sectors — visits all 5 before reset  │
│    HOLD_SAFE = loiter circle r=15m, ω=0.3 rad/s            │
└───────────────────────────┬─────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────┐
│               PX4 SITL + GAZEBO                             │
│       /fmu/in/trajectory_setpoint  @  20 Hz                 │
└─────────────────────────────────────────────────────────────┘
         ↑                                        │
         └──── Feedback: pos, battery, EKF ───────┘

Side outputs (every 0.5s):
  ├── /tmp/uav_dashboard_state.json  →  uav_dashboard.py
  └── ~/uav_dataset/flight_*.jsonl   →  uav_label_dataset.py
```

### 7 Mission Intents (LLM Output)

| # | Intent | Trigger Condition | Behavior |
|---|--------|-------------------|----------|
| 1 | `HOLD_SAFE` | EKF degraded / LLM uncertain | Loiter circle r=15m, ω=0.3 rad/s |
| 2 | `CONTINUE_ASSIGNED_TASK` | Resume after interruption | Fly back to assigned sector |
| 3 | `SEARCH_NEXT_PRIORITY` | Default mission | Fly to next **unvisited** sector by priority |
| 4 | `INSPECT_TARGET_REPORT` | Unconfirmed target nearby | Confirm nearest victim target |
| 5 | `COMMS_REACQUIRE` | Signal lost / post-loss re-entry | Fly to [30, 30, -25] comms point |
| 6 | `RETURN_TO_BASE` | Battery < 25% | RTB to [0, 0, -5] |
| 7 | `ABORT_AND_SAFE` | Battery < 15% critical | Emergency land immediately |

> **Hard safety gate:** Intents 6 and 7 are also forced deterministically before the LLM runs when battery is critical. The LLM cannot override these.

### Search Map

```
         N (+x)
          ↑
          │         SECTOR_A [100, 0]   priority 1
          │
          │  SECTOR_E                   SECTOR_B
          │  [80, 80]                   [50, 100]   priority 2
W ────────┼──────────────────────────────────────── E
(-y)      │                                       (+y)
    SECTOR_D      BASE
    [-100, 0]    [0, 0]    priority 4
          │
          │     SECTOR_C [0, -100]      priority 3
          ↓
         S (-x)

Altitude: NED frame — negative z = UP
  -5m  = base hover altitude
  -10m = search altitude
  -25m = comms recovery point (higher = better signal)
```

<br>

---

## 🧰 Prerequisites & Software Stack

| Software | Version | Purpose | Official Docs |
|----------|---------|---------|---------------|
| **Ubuntu** | 22.04 LTS | Operating System | [ubuntu.com/download](https://ubuntu.com/download/desktop) |
| **ROS 2 Humble** | Humble Hawksbill | Robot middleware & topic comms | [docs.ros.org/en/humble](https://docs.ros.org/en/humble/Installation.html) |
| **PX4 Autopilot** | v1.14+ | Flight controller firmware (SITL) | [docs.px4.io](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html) |
| **Gazebo** | Garden / Harmonic | 3D drone physics simulation | [gazebosim.org/docs](https://gazebosim.org/docs) |
| **Micro XRCE-DDS Agent** | latest | ROS 2 ↔ PX4 uXRCE bridge over UDP | [github.com/eProsima](https://github.com/eProsima/Micro-XRCE-DDS-Agent) |
| **QGroundControl** | Stable | Ground control station & telemetry | [qgroundcontrol.com](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) |
| **Ollama** | latest | Local LLM runtime (fully offline) | [ollama.com/download](https://ollama.com/download) |
| **LLaMA3** | llama3:latest (8B) | AI mission decision model | [ollama.com/library/llama3](https://ollama.com/library/llama3) |
| **Python** | 3.10+ | Main control scripts | [python.org/downloads](https://www.python.org/downloads/) |
| **px4_msgs** | ROS 2 package | PX4 ROS 2 message definitions | [github.com/PX4/px4_msgs](https://github.com/PX4/px4_msgs) |
| **px4_ros_com** | ROS 2 package | PX4 ROS 2 bridge utilities | [github.com/PX4/px4_ros_com](https://github.com/PX4/px4_ros_com) |

<br>

---

## 📦 Installation Guide

### 1. ROS 2 Humble

> 📖 Official docs: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop

# Install colcon build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init && rosdep update
```

**Verify:**
```bash
source /opt/ros/humble/setup.bash
ros2 --version
# Expected: ros2 cli version X.X.X (humble)
```

---

### 2. PX4 Autopilot + Gazebo SITL

> 📖 Official docs: https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html

```bash
# Clone PX4 with all submodules (~2.5 GB)
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Run the official Ubuntu setup script
# Installs all toolchains, Gazebo, and dependencies automatically
bash ./Tools/setup/ubuntu.sh

# Reboot after setup to apply udev rules
sudo reboot
```

**Verify — test launch with X500 quadrotor:**
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
# ✅ Gazebo should open with X500 drone
# ✅ PX4 console shows: INFO [commander] Ready for takeoff!
# Press Ctrl+C to stop
```

---

### 3. Micro XRCE-DDS Agent

> The **critical bridge** between PX4 and ROS 2. Without it, no `/fmu/` topics appear in ROS 2.
>
> 📖 Official docs: https://micro-xrce-dds.docs.eprosima.com/en/latest/

```bash
# Install build dependencies
sudo apt install -y cmake g++ python3-pip git

# Clone and build from source
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/
```

**Verify:**
```bash
MicroXRCEAgent --help
# Expected: Usage: MicroXRCEAgent <transport> [options]
```

---

### 4. QGroundControl

> 📖 Official docs: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

```bash
# Install required system libraries
sudo apt install -y libfuse2 libxcb-xinerama0 libxkbcommon-x11-0

# Download AppImage to home directory
cd ~
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage

# Make it executable
chmod +x ~/QGroundControl-x86_64.AppImage
```

**Verify:**
```bash
~/QGroundControl-x86_64.AppImage
# ✅ QGC window opens and auto-connects to simulation on localhost:14550
```

---

### 5. Ollama + LLaMA3

> This project uses **Ollama** to run LLaMA3 **entirely offline** — no API keys, no internet during flight.
>
> 📖 Ollama install: https://ollama.com/download
> 📖 LLaMA3 model: https://ollama.com/library/llama3

**Install Ollama:**
```bash
curl -fsSL https://ollama.com/install.sh | sh
```

**Verify Ollama is installed:**
```bash
ollama --version
# Expected: ollama version 0.x.x
```

**Pull the exact LLaMA3 model used in this project:**
```bash
# One-time download ~4.7 GB
ollama pull llama3
```

**Verify the model works:**
```bash
ollama run llama3 "Reply with the word READY only."
# Expected output: READY
```

---

### 6. Python Dependencies

```bash
pip install pynput --break-system-packages
```

> `curses` is part of the Python standard library — no install needed for the dashboard.

---

### 7. Build the ROS 2 Workspace

```bash
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash
```

**Verify:**
```bash
ros2 pkg list | grep -E "px4_msgs|px4_ros_com|px4_offboard|uav_control"
# Expected:
# px4_msgs
# px4_offboard
# px4_ros_com
# uav_control
```

<br>

---

## 🚀 Running the System

Open **4 separate terminals**. Run each **in order** — sequence matters.

---

### Terminal 1 — Micro XRCE-DDS Agent (Start First)

```bash
cd ~/Micro-XRCE-DDS-Agent/build
MicroXRCEAgent udp4 -p 8888
```

> Bridges PX4 uORB topics → ROS 2 over UDP port 8888.
> Keep this running. Connection logs appear once PX4 starts.

---

### Terminal 2 — PX4 SITL + Gazebo

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

> Launches X500 quadrotor in Gazebo. Wait until you see:
> ```
> INFO  [commander] Ready for takeoff!
> ```

---

### Terminal 3 — Main Control Node

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
python3 autonomous_uav_final.py
```

> Drone arms and takes off automatically.
> Fly manually with keyboard arrows.
> After **15s of no input** → LLM takes autonomous control and begins visiting all 5 sectors in priority order.

---

### Terminal 4 — Live Operator Dashboard

```bash
python3 uav_dashboard.py
```

> Shows battery bar, EKF, signal status, full LLM reasoning, confidence, and flight log — all live.
> Press `Q` to close. Drone keeps flying.

---

### After Flight — Label Dataset

```bash
python3 uav_label_dataset.py
```

> Reviews each LLM decision. Mark correct/wrong, write better reasoning. Export as fine-tuning data.

---

### ✅ Quick-Start (Copy-Paste)

```bash
# Terminal 1 — DDS Bridge
cd ~/Micro-XRCE-DDS-Agent/build && MicroXRCEAgent udp4 -p 8888

# Terminal 2 — PX4 + Gazebo
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 3 — Main Control Node
cd ~/ros_ws && source /opt/ros/humble/setup.bash && source install/local_setup.bash && python3 autonomous_uav_final.py

# Terminal 4 — Dashboard
python3 uav_dashboard.py
```

> ⚠️ **Always start in order: T1 → T2 → T3 → T4**

<br>

---

## ⌨️ Keyboard Controls & Test Keys

### Manual Flight Controls

| Key | Action |
|-----|--------|
| `↑` | Move forward (North) |
| `↓` | Move backward (South) |
| `←` | Move left (West) |
| `→` | Move right (East) |
| `W` | Climb (increase altitude) |
| `S` | Descend (decrease altitude) |

### Developer Test Keys

| Key | Action | What Happens |
|-----|--------|--------------|
| `T` | Force EKF → 0.2 **(locked)** | Safety gate fires instantly → HOLD_SAFE loiter. LLM skipped. Stays locked until `R`. |
| `Y` | Force Battery → 10% **(locked)** | Safety gate fires instantly → RTB. LLM skipped. Stays locked until `R`. |
| `R` | Reset both to normal | Releases `T` and `Y` locks. Sensors read from PX4 again. |
| `Z` | Simulate 5s signal loss | Tests full signal loss → regain → re-entry LLM decision flow. |
| `F` | Force LLM re-query now | Skips 30s cooldown — immediate new decision. |
| `H` | Show mission stats | Prints dataset count, visited sectors, remaining sectors. |
| `B` | Show battery status | Prints voltage, current, and percentage. |
| `1`–`7` | Force specific intent | Bypasses LLM entirely — directly activates that intent. |

> **Note on T and Y:** Overrides are **locked** until you press `R`. The ROS sensor callbacks are blocked from reading real PX4 values while the lock is active. You will see `[EKF_OVR]` or `[BATT_OVR]` in the status line confirming the lock. This ensures the forced value holds stable across multiple LLM queries.

<br>

---

## 🧠 How the LLM Brain Works

### Decision Flow

```
Every 30s or on trigger event:

1. trigger_llm() called
         │
         ▼
2. Hard safety gate:
   Battery < 15%?  →  RTB immediately  (LLM skipped entirely)
   EKF < 0.4?      →  HOLD_SAFE immediately  (LLM skipped entirely)
         │
         ▼  (only if sensors are within safe limits)
3. Background thread starts — main loop keeps running at 20Hz
   Full input snapshot taken before query begins
         │
         ▼
4. Prompt sent to Ollama (llama3):
   Includes battery, EKF, full flight log, all sector visit
   status (✅ VISITED / ⬜ NOT YET VISITED), signal loss
   context block if a loss just occurred
         │
         ▼
5. LLM responds with JSON:
   {"intent":3, "reasoning":"...", "confidence":86, "next_checkpoint":"..."}
         │
         ▼
6. Confidence gate applied:
   ≥ 70%  →  execute
   50-69% →  execute with warning logged
   < 50%  →  ignore LLM, use deterministic safe fallback
         │
         ▼
7. Waypoint generated, drone flies
   Decision recorded to dataset file
```

### Sector Visiting Logic

The drone uses a `visited_sectors` **set** to track all visited sectors. A sector is only marked visited when the drone **physically arrives** — not when the decision is made. If the drone is interrupted mid-flight, it retries that sector. When all 5 are done the set resets and a new sweep begins automatically.

```
Sweep 1:  A → B → C → D → E → [all done, reset]
Sweep 2:  A → B → C → D → E → [reset again]
```

### Signal Loss Re-Entry

When signal is regained after a loss, the LLM gets a dedicated context block injected into its prompt:

```
=== SIGNAL LOSS EVENT — ADDRESS THIS BEFORE DECIDING ===
  Lost for         : 8.2s
  Battery cost     : 2.1% — negligible
  Position drift   : 1.4m — minimal
  EKF at regain    : 0.88 — GOOD
  Loss #           : 1 this flight
  Interrupted task : SEARCH_NEXT_PRIORITY → SECTOR_B
Consider: Is it safe to resume? Is the drift acceptable?
```

The LLM explicitly addresses this before deciding whether to resume the interrupted task or change plan. If this is the 2nd+ loss a pattern warning is added.

<br>

---

## 📊 Dataset Collection & Fine-Tuning

Every flight automatically records decisions to `~/uav_dataset/`. One file per session named by timestamp.

### What Gets Recorded Per Decision

```json
{
  "id": "flight_20260221_143022_003",
  "input": {
    "battery_pct": 67.3,
    "ekf_confidence": 0.91,
    "sectors_visited": ["SECTOR_A"],
    "sectors_remaining": ["SECTOR_B", "SECTOR_C", "SECTOR_D", "SECTOR_E"],
    "post_signal_loss_reentry": true,
    "event_log_at_decision": ["[T+89s] Arrived SECTOR_A", "..."],
    "last_loss_duration_s": 8.2,
    "last_loss_battery_cost": 2.1
  },
  "llm_output": {
    "intent": 3,
    "intent_name": "SEARCH_NEXT_PRIORITY",
    "reasoning": "Battery healthy. Signal loss brief. Resume search.",
    "confidence": 86,
    "response_time_seconds": 4.2,
    "fallback_used": false,
    "was_reentry_decision": true
  },
  "outcome": {
    "waypoint_reached": true,
    "battery_at_arrival": 61.2,
    "time_to_arrive_seconds": 47.0,
    "dwell_completed": true
  },
  "label": {
    "correct": null,
    "correct_intent": null,
    "correct_reasoning": null
  }
}
```

### Labeling Workflow

```bash
# After a flight session
python3 uav_label_dataset.py

# For each decision:
#   Y = correct
#   N = wrong  (prompts for correct intent + better reasoning)
#   S = skip
#   Q = quit and save progress

# Show stats only (no labeling)
python3 uav_label_dataset.py --stats

# Export labeled data to fine-tuning format
python3 uav_label_dataset.py --export
# Output: ~/uav_dataset/export/flight_*_training.jsonl
# Format: Unsloth SFT compatible
# {"instruction":"...", "input":"<situation>", "output":"<correct JSON>"}
```

After ~200 labeled examples you have enough to LoRA fine-tune LLaMA3 on your specific mission scenarios and failure cases.

<br>

---

## 📁 Project Structure

```
ros_ws/
├── autonomous_uav_final.py      # 🔑 MAIN SCRIPT — run this
│                                #    All 5 stages in one file
│                                #    State machine, LLM brain, safety gate,
│                                #    signal loss memory, dataset collector
│
├── uav_dashboard.py             # 📊 Live operator display (curses)
│                                #    Run in a separate terminal
│                                #    Reads /tmp/uav_dashboard_state.json
│
├── uav_label_dataset.py         # 🏷  Post-flight labeling tool
│                                #    Labels decisions correct/wrong
│                                #    Exports fine-tuning JSONL
│
├── ~/uav_dataset/               # 📁 Auto-created on first run
│   ├── flight_20260221_*.jsonl  #    One file per flight session
│   └── export/
│       └── *_training.jsonl     #    Labeled fine-tuning data
│
└── src/
    ├── px4_msgs/                # PX4 ↔ ROS 2 message definitions
    │   └── msg/                 # VehicleLocalPosition, BatteryStatus, etc.
    ├── px4_offboard/            # Offboard control package
    ├── px4_ros_com/             # PX4 ↔ ROS 2 bridge utilities
    │   └── src/lib/
    │       └── frame_transforms.cpp  # NED ↔ ENU coordinate transforms
    └── uav_control/             # Autonomous decision package
```

### Key Files at a Glance

| File | Role |
|------|------|
| `autonomous_uav_final.py` | **Main script** — all logic, run this |
| `uav_dashboard.py` | **Live display** — separate terminal, Q to close |
| `uav_label_dataset.py` | **Labeling tool** — run after flights |
| `px4_msgs/msg/` | Message definitions for all PX4 ↔ ROS 2 topics |
| `px4_ros_com/src/lib/frame_transforms.cpp` | NED ↔ ENU coordinate frame utilities |

<br>

---

## 🔧 Troubleshooting

**Gazebo doesn't open or freezes:**
```bash
pkill -f gz && pkill -f gzserver && pkill -f gzclient
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```

**`/fmu/` ROS 2 topics not appearing:**
```bash
# Always start DDS agent BEFORE PX4
ros2 topic list | grep fmu
# Must show /fmu/in/ and /fmu/out/ topics
# If empty → stop everything, restart T1 then T2
```

**`colcon build` fails on px4_msgs:**
```bash
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select px4_msgs
colcon build --symlink-install
```

**Ollama not responding / LLM silent:**
```bash
systemctl status ollama
sudo systemctl restart ollama
curl http://localhost:11434/api/generate \
  -d '{"model":"llama3","prompt":"say READY","stream":false}'
```

**Script exits immediately after sourcing:**
```bash
# Run both source commands every new terminal
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/local_setup.bash
python3 autonomous_uav_final.py
```

**QGC can't find drone:**
> PX4 SITL must be running first. QGC auto-connects to `localhost:14550` UDP. No manual config needed for SITL.

**Dashboard says "command not found":**
```bash
# Always run with python3 explicitly
python3 uav_dashboard.py

# If curses error — terminal may be too small, try resizing window or:
TERM=xterm python3 uav_dashboard.py
```

**Dashboard says "Waiting for UAV system to start":**
```bash
# Make sure autonomous_uav_final.py is running first
# The dashboard reads /tmp/uav_dashboard_state.json written by the node
ls /tmp/uav_dashboard_state.json
```

**T or Y key snaps back to normal immediately:**
> Fixed in `autonomous_uav_final.py`. Overrides are locked until you press `R`. Look for `[EKF_OVR]` or `[BATT_OVR]` in the status line — these confirm the lock is active. If you don't see them make sure you're running `autonomous_uav_final.py` and not an older stage file.

**Drone only visits 2 sectors and keeps toggling:**
> Fixed in `autonomous_uav_final.py` by replacing `last_sector_id` (single int) with `visited_sectors` (set). Make sure you're running the final version.

**Battery critical but LLM picks a sector instead of RTB:**
> Fixed by the hard safety gate in `trigger_llm()`. When battery < 15% the LLM is bypassed entirely and RTB is forced before Ollama is called. Make sure you're running `autonomous_uav_final.py`.

<br>

---

## 👥 Contributors

| GitHub | Role |
|--------|------|
| [@OrgAccount](https://github.com) | Primary development & ROS 2 integration |
| [@PersonalAccount](https://github.com) | Autonomous architecture & LLM integration |

> **Institution:** University of Michigan - Dearborn
> **Project:** UAV Triage Architecture — Autonomous Systems Research
> **Year:** 2026

<br>

---

## 📄 License

Developed for academic research purposes.
University of Michigan - Dearborn © 2026

---

*Stack: ROS 2 Humble · PX4 v1.14 · Gazebo · Micro XRCE-DDS · Ollama · LLaMA3 8B · Python 3.10 · Ubuntu 22.04*
