# 🚁 EGLe — Edge-Deployed LLM Autonomy for UAV Triage

> **EGLe** *(Edge-deployed Generative Language engine)* is a fully autonomous UAV decision-making framework that deploys a local or edge LLM to reason like a real pilot during communication loss — with full flight memory, signal loss re-entry logic, hard safety gates, live operator dashboard, and automatic research dataset collection.
>
> Simulated in **Gazebo** via **PX4 SITL + ROS 2 Humble**. Supports two deployment modes: **Local GCS** (LLaMA3 8B on laptop) and **Raspberry Pi 5 Edge** (LLaMA 3.2 1B over WiFi).

<br>

---

## 📋 Table of Contents

- [Project Overview](#-project-overview)
- [Hardware Configurations](#️-hardware-configurations)
- [System Architecture](#-system-architecture)
- [Prerequisites & Software Stack](#-prerequisites--software-stack)
- [Installation Guide](#-installation-guide)
  - [1. ROS 2 Humble](#1-ros-2-humble)
  - [2. PX4 Autopilot + Gazebo SITL](#2-px4-autopilot--gazebo-sitl)
  - [3. Micro XRCE-DDS Agent](#3-micro-xrce-dds-agent)
  - [4. QGroundControl](#4-qgroundcontrol)
  - [5. Ollama + LLaMA3 (Local GCS)](#5-ollama--llama3-local-gcs)
  - [6. Raspberry Pi 5 Edge Setup](#6-raspberry-pi-5-edge-setup)
  - [7. Python Dependencies](#7-python-dependencies)
  - [8. Build the ROS 2 Workspace](#8-build-the-ros-2-workspace)
- [Running the System](#-running-the-system)
  - [Config A — Local GCS](#config-a--local-gcs)
  - [Config B — Pi 5 Edge](#config-b--raspberry-pi-5-edge)
- [Keyboard Controls & Test Keys](#️-keyboard-controls--test-keys)
- [How the LLM Brain Works](#-how-the-llm-brain-works)
- [5-Stage Evolution](#-5-stage-evolution)
- [uav\_dataset/ Structure & Log Files](#-uav_dataset-structure--log-files)
- [Research Results Summary](#-research-results-summary)
- [Project File Structure](#-project-file-structure)
- [Troubleshooting](#-troubleshooting)
- [Contributors](#-contributors)

<br>

---

## 🧠 Project Overview

This project implements a **fully autonomous UAV triage architecture** where the drone:

- Flies manually via keyboard in real-time at 20 Hz
- Switches to **autonomous mode** after 15 seconds of no keyboard input
- Uses a **local or edge LLM** to reason like a real pilot — with flight memory, transparent reasoning, confidence scores, and signal loss handling
- Visits **all 5 search sectors** in priority order before resetting for a new sweep
- **Remembers signal loss events** and makes smart re-entry decisions after comms restore
- Shows a **live operator dashboard** in a second terminal with full LLM reasoning visible
- **Records every decision** to a `.jsonl` dataset file with full metrics for research analysis
- Hard safety gate bypasses the LLM entirely for critical battery/EKF states

All simulation runs **100% locally** using PX4 SITL + Gazebo. The edge configuration communicates with a Raspberry Pi 5 over WiFi — no cloud required.

<br>

---

## ⚙️ Hardware Configurations

Two configurations were evaluated for the EGLe paper:

| Parameter | Config A — Local GCS | Config B — Pi 5 Edge |
|-----------|---------------------|----------------------|
| LLM Model | LLaMA3 8B | LLaMA 3.2 1B |
| Hardware | Laptop / workstation | Raspberry Pi 5 (16GB RAM) |
| Inference | `ollama` localhost | `ollama` on Pi over WiFi |
| Mean latency | 3.241s (σ=0.714) | 52.642s (σ=11.304) |
| Mean confidence | 82.8% | 90.0% |
| Network | None (local) | WiFi 10.0.0.0/24 |
| Cooling required | No | **Yes — heatsink recommended** |
| Script | `autonomous_uav_final.py` | `autonomous_uav_pi_edge_short.py` |

> ⚠️ **Pi 5 Thermal Warning:** Without active cooling, the Pi 5 reaches ~78.5°C during inference (throttle threshold: 80°C). A passive heatsink (~$5–10) is **strongly recommended** for sustained operation.

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
│  ┌──────────────────────┐  ┌───────────────────────────┐   │
│  │  CONFIG A (Local)    │  │  CONFIG B (Pi 5 Edge)     │   │
│  │  LLaMA3 8B           │  │  LLaMA 3.2 1B             │   │
│  │  ollama localhost    │  │  ollama Pi over WiFi      │   │
│  │  ~3.2s latency       │  │  ~52.6s latency           │   │
│  └──────────────────────┘  └───────────────────────────┘   │
│    Output: intent + reasoning + confidence + checkpoint      │
│    Confidence gate: ≥70 execute | 50-69 warn | <50 fallback  │
└───────────────────────────┬─────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────┐
│                WAYPOINT GENERATOR                           │
│    Converts intent → battery-aware [x, y, z]               │
│    Tracks visited sectors — visits all 5 before reset       │
│    HOLD_SAFE = loiter circle r=15m, ω=0.3 rad/s             │
└───────────────────────────┬─────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────┐
│               PX4 SITL + GAZEBO                             │
│       /fmu/in/trajectory_setpoint  @  20 Hz                 │
└─────────────────────────────────────────────────────────────┘

Side outputs (every 0.5s):
  ├── /tmp/uav_dashboard_state.json  →  uav_dashboard.py
  └── ~/uav_dataset/flight_*.jsonl   →  research dataset
```

### 7 Mission Intents

| # | Intent | Trigger | Behavior |
|---|--------|---------|----------|
| 1 | `HOLD_SAFE` | EKF degraded / uncertainty | Loiter circle r=15m, ω=0.3 rad/s |
| 2 | `CONTINUE_ASSIGNED_TASK` | Resume interrupted transit | Fly back to assigned sector |
| 3 | `SEARCH_NEXT_PRIORITY` | Default mission | Fly to next unvisited sector |
| 4 | `INSPECT_TARGET_REPORT` | Unconfirmed target nearby | Confirm nearest victim target |
| 5 | `COMMS_REACQUIRE` | Signal loss re-entry | Fly to [30, 30, -25] comms point |
| 6 | `RETURN_TO_BASE` | Battery < 25% | RTB to [0, 0, -5] |
| 7 | `ABORT_AND_SAFE` | Battery < 15% critical | Emergency land immediately |

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

| Software | Version | Purpose |
|----------|---------|---------|
| **Ubuntu** | 22.04 LTS | Operating System |
| **ROS 2 Humble** | Humble Hawksbill | Robot middleware & topic comms |
| **PX4 Autopilot** | v1.14+ | Flight controller firmware (SITL) |
| **Gazebo** | Garden / Harmonic | 3D drone physics simulation |
| **Micro XRCE-DDS Agent** | latest | ROS 2 ↔ PX4 uXRCE bridge over UDP |
| **QGroundControl** | Stable | Ground control station & telemetry |
| **Ollama** | latest | Local LLM runtime (fully offline) |
| **Python** | 3.10+ | Node scripts |
| **Raspberry Pi OS** | Bookworm 64-bit | Pi edge node OS (Config B only) |

<br>

---

## 🔧 Installation Guide

### 1. ROS 2 Humble

```bash
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-humble-desktop \
  python3-colcon-common-extensions -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. PX4 Autopilot + Gazebo SITL

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
pip install --user -r Tools/setup/requirements.txt
make px4_sitl gz_x500   # First build takes ~10 min
```

### 3. Micro XRCE-DDS Agent

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make
sudo make install
sudo ldconfig /usr/local/lib/
```

### 4. QGroundControl

```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav \
  gstreamer1.0-gl libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 -y
# Download AppImage from https://docs.qgroundcontrol.com
chmod +x QGroundControl.AppImage && ./QGroundControl.AppImage
```

### 5. Ollama + LLaMA3 (Local GCS)

```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Pull both models
ollama pull llama3          # Config A — 8B model
ollama pull llama3.2:1b     # Config B — also used on Pi

# Test
curl http://localhost:11434/api/generate \
  -d '{"model":"llama3","prompt":"Say READY","stream":false}'
```

### 6. Raspberry Pi 5 Edge Setup

> **Requirements:** Raspberry Pi 5 (8GB or 16GB RAM), Raspberry Pi OS Bookworm 64-bit, 27W USB-C PSU, passive heatsink recommended.

#### 6a. Install Ollama on Pi

```bash
# SSH into Pi
ssh <user>@<pi-ip>

# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Pull 1B model (takes ~5 min on Pi)
ollama pull llama3.2:1b

# Test
curl http://localhost:11434/api/generate \
  -d '{"model":"llama3.2:1b","prompt":"Say READY","stream":false}'
```

#### 6b. Expose Ollama over WiFi network

```bash
# Create systemd override to bind on all interfaces
sudo mkdir -p /etc/systemd/system/ollama.service.d/
sudo nano /etc/systemd/system/ollama.service.d/override.conf
```

Paste the following:

```ini
[Service]
Environment="OLLAMA_HOST=0.0.0.0:11434"
Environment="OLLAMA_NUM_PARALLEL=1"
Environment="OLLAMA_NUM_THREADS=2"
Environment="OLLAMA_KEEP_ALIVE=30s"
CPUQuota=200%
```

```bash
sudo systemctl daemon-reload
sudo systemctl restart ollama

# Verify from laptop — should return model list
curl http://<pi-ip>:11434/api/tags
```

> **Why `CPUQuota=200%` + `OLLAMA_NUM_THREADS=2`?**
> Limits inference to 2 CPU cores maximum. Without this the Pi 5 draws excessive current from USB-C chargers causing brownouts, and peak temperature is reduced from ~80°C to ~70°C.

#### 6c. Find Pi IP address

```bash
# On Pi
hostname -I

# Or from laptop
arp -a | grep -i raspberry
ping raspberrypi.local
```

#### 6d. Set Pi IP in edge script

```bash
nano ~/UAV-Triage/ros_ws/src/uav_control/uav_control/autonomous_uav_pi_edge_short.py
# Find line: PI_OLLAMA_URL = "http://10.0.0.119:11434/api/generate"
# Replace 10.0.0.119 with your Pi's IP
```

#### 6e. Pi Hardware Monitor (run during every experiment)

```bash
# SSH into Pi in a separate terminal and run this BEFORE starting the UAV script
# Leave it running the entire session — it auto-saves to ~/pi_hardware_log_*.txt
while true; do
  echo "$(date) | CPU: $(top -bn1 | grep 'Cpu(s)' | awk '{print $2}')% | \
RAM: $(free -m | awk 'NR==2{printf "%s/%s MB", $3,$2}') | \
TEMP: $(vcgencmd measure_temp)"
  sleep 5
done | tee ~/pi_hardware_log_$(date +%Y%m%d_%H%M%S).txt
```

Pull logs to laptop after experiments:

```bash
scp <user>@<pi-ip>:~/pi_hardware_log_*.txt ~/uav_dataset/
```

### 7. Python Dependencies

```bash
pip3 install --break-system-packages rclpy numpy
# No extra packages needed — edge script uses only Python stdlib (urllib, json)
```

### 8. Build the ROS 2 Workspace

```bash
cd ~/UAV-Triage/ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash
echo "source ~/UAV-Triage/ros_ws/install/local_setup.bash" >> ~/.bashrc
```

<br>

---

## 🚀 Running the System

> **Golden rule — always start in this order:**
> `Pi Ollama (if edge) → DDS Bridge → PX4/Gazebo → UAV Script → Dashboard`

---

### Config A — Local GCS

Open **4 terminals** on your laptop:

**Terminal 1 — DDS Bridge**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 — PX4 + Gazebo**
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
# Wait for: INFO [commander] Ready for takeoff!
```

**Terminal 3 — UAV Autonomous Node**
```bash
source /opt/ros/humble/setup.bash
source ~/UAV-Triage/ros_ws/install/local_setup.bash
cd ~/UAV-Triage/ros_ws
python3 src/uav_control/uav_control/autonomous_uav_final.py
```

**Terminal 4 — Live Dashboard (optional)**
```bash
source /opt/ros/humble/setup.bash
source ~/UAV-Triage/ros_ws/install/local_setup.bash
python3 ~/UAV-Triage/ros_ws/src/uav_control/uav_control/uav_dashboard.py
```

---

### Config B — Raspberry Pi 5 Edge

**Step 1 — Verify Pi before starting**
```bash
# From laptop
ping <pi-ip>
ssh <user>@<pi-ip> "systemctl is-active ollama && vcgencmd measure_temp"
# Must show: active  AND  temp below 45°C before first run
```

**Step 2 — Pi SSH Terminal — Start hardware monitor**
```bash
# SSH into Pi — leave this running the entire experiment
while true; do
  echo "$(date) | CPU: $(top -bn1 | grep 'Cpu(s)' | awk '{print $2}')% | \
RAM: $(free -m | awk 'NR==2{printf "%s/%s MB", $3,$2}') | \
TEMP: $(vcgencmd measure_temp)"
  sleep 5
done | tee ~/pi_hardware_log_$(date +%Y%m%d_%H%M%S).txt
```

**Terminal 1 — DDS Bridge**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 — PX4 + Gazebo**
```bash
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```

**Terminal 3 — UAV Edge Node**
```bash
source /opt/ros/humble/setup.bash
source ~/UAV-Triage/ros_ws/install/local_setup.bash
cd ~/UAV-Triage/ros_ws
python3 src/uav_control/uav_control/autonomous_uav_pi_edge_short.py
```

**Between each run — wait for Pi to cool**
```bash
ssh <user>@<pi-ip> "vcgencmd measure_temp"
# Wait until temp < 45°C before starting next run
```

---

### Expected Terminal Output

```
[LOG] ARM command sent.
[LOG] Switched to AUTONOMOUS. No input: 15s.
[LLM] 🧠 Thinking...
[EDGE] Pi inference | model=llama3.2:1b | prompt_tokens=538 |
       output_tokens=46 | inference=13.84s | tok/s=3.3 | network_rtt=51.88s
╔══════════════════════════════════════════════════════════╗
║  🧠 PILOT BRAIN DECISION                                 ║
║  Intent    : [3] SEARCH_NEXT_PRIORITY                    ║
║  Target    : SECTOR_A                                    ║
║  Confidence: 90%  ✅ HIGH                                ║
╚══════════════════════════════════════════════════════════╝
```

<br>

---

## ⌨️ Keyboard Controls & Test Keys

| Key | Action | Notes |
|-----|--------|-------|
| `W/A/S/D` | Manual flight | Cancels autonomous mode timer |
| `Space` | Land | Emergency stop |
| `Z` | Simulate 5s signal loss | Tests re-entry logic |
| `T` | Force EKF → 0.2 (critical) | Tests safety gate — HOLD_SAFE fires |
| `Y` | Force Battery → 10% (critical) | Tests safety gate — RTB fires |
| `R` | Reset T/Y overrides | Restores normal sensor values |
| `F` | Force immediate LLM re-query | Skips dwell timer |
| `1–7` | Force specific intent | Manual intent injection |
| `H` | Show dataset stats | Decisions recorded this session |
| `Q` | Quit | Exits cleanly |

> **Testing safety gates:** Press `T` or `Y` → confirm `[SAFETY] gate fired` in logs → press `R` to restore. LLM is bypassed — gate fires in <1ms.

<br>

---
## 🎮 Gamepad Controller (ShanWan Readgear)
Run: python3 src/uav_control/uav_control/autonomous_uav_controller.py

| Control        | Action                  |
|----------------|-------------------------|
| Left stick XY  | Fly forward/back/strafe |
| Right stick Y  | Altitude up/down        |
| X button       | Battery spoof → 10%     |
| O button       | EKF spoof → 0.2         |
| Triangle       | Reset both overrides     |
| L1             | Force LLM re-query      |

## 🗺️ Gazebo Worlds
- egle_survey.sdf  — flat world with 5 colored sector markers
- egle_baylands.sdf — realistic baylands with sector markers

Launch:
PX4_GZ_WORLD=egle_survey make px4_sitl gz_x500

## 📍 QGC Waypoint Upload
python3 upload_waypoints.py
## 🧠 How the LLM Brain Works

### Compressed Prompt (edge config, ~380 tokens)

```
UAV pilot brain. Operator unavailable. Decide next action.

STATE: pos=[12.3, -4.1, -10.0] batt=50%(GOOD) ekf=0.80(GOOD) wp=SECTOR_A
LOG(last 5):
  [T+1m21s] Arrived at SECTOR_A. Battery: 50.0%
  [T+0m15s] Switched to AUTONOMOUS

MISSION: visited=[SECTOR_A] remaining=[SECTOR_B,SECTOR_C,SECTOR_D,SECTOR_E]

ACTIONS:
1=HOLD_SAFE 2=CONTINUE_ASSIGNED_TASK 3=SEARCH_NEXT_PRIORITY
4=INSPECT_TARGET 5=COMMS_REACQUIRE 6=RETURN_TO_BASE 7=ABORT

RULES: batt<25%→RTB, batt<15%→ABORT, ekf<0.4→no far flight, safety first.

Reply ONLY valid JSON:
{"intent":<1-7>,"reasoning":"<brief>","confidence":<0-100>,"next_checkpoint":"<when>"}
```

### Output JSON

```json
{
  "intent": 3,
  "reasoning": "Battery 50% healthy. EKF 0.80 solid. SECTOR_A visited. SECTOR_B is next priority.",
  "confidence": 90,
  "next_checkpoint": "Arrive SECTOR_B or battery drops below 25%"
}
```

### Confidence Gate

| Confidence | Action |
|------------|--------|
| ≥ 70% | Execute decision ✅ |
| 50–69% | Execute with warning logged ⚠️ |
| < 50% | Ignore — use `HOLD_SAFE` fallback ❌ |

### Hard Safety Gate (runs before LLM, always)

| Condition | Action | LLM Called? |
|-----------|--------|-------------|
| Battery < 15% | Force `ABORT_AND_SAFE` | No |
| Battery < 25% | Force `RETURN_TO_BASE` | No |
| EKF < 0.4 | Force `HOLD_SAFE` | No |
| All clear | Normal LLM query | Yes |

<br>

---

## 🆕 5-Stage Evolution

### Stage 1 — Event Log System
Every significant event is timestamped and passed to every LLM query, giving the model full flight memory to avoid repeating decisions.

### Stage 2 — Pilot Brain Prompt
Replaced rigid rules with a genuine reasoning prompt. Structured JSON output with `intent`, `reasoning`, `confidence`, `next_checkpoint`. Confidence gate decides execute, warn, or fallback.

### Stage 3 — Signal Loss Memory
Re-entry context block encodes loss duration, battery cost, position drift, and pre-loss intent — enabling smart post-outage reasoning.

### Stage 4 — Live Operator Dashboard
`uav_dashboard.py` renders a live terminal display with battery bar, EKF status, full LLM reasoning, and last 8 log entries — all updating in real time.

### Stage 5 — Dataset Collection
Every LLM decision recorded to `.jsonl` with full input, output, edge metrics, and outcome tracking for research analysis and fine-tuning.

<br>

---

## 📁 uav\_dataset/ Structure & Log Files

```
uav_dataset/
│
├── flight_YYYYMMDD_HHMMSS.jsonl          # One file per flight session
│                                          # Auto-created on first decision
│                                          # Local config:  ~12 decisions/run
│                                          # Edge config:   ~5  decisions/run
│
├── local_runs/                            # Config A runs (Local GCS)
│   └── flight_202602280302xx.jsonl       # 5 sessions × 12 decisions = 60 total
│
├── run_YYYYMMDD_HHMMSS_local_terminal.log # Full terminal output — local runs
│                                          # Contains: LLM responses, safety gate
│                                          # events, JSON failures, status lines
│
├── run_YYYYMMDD_HHMMSS_edge_terminal.log  # Full terminal output — edge runs
│                                          # Contains: [EDGE] inference metrics,
│                                          # timeout events, Pi RTT per query
│
├── pi_hardware_log_YYYYMMDD_HHMMSS.txt   # Raspberry Pi hardware monitor
│                                          # Sampled every 5s during flights
│                                          # Format: DATE | CPU: X% |
│                                          #         RAM: X/X MB | TEMP: X°C
│                                          # Used for: Section E paper metrics
│
├── analyze_paper_data.py                  # Research analysis script
│                                          # Reads all .jsonl + pi_hardware logs
│                                          # Outputs latency stats, intent dist,
│                                          # signal loss accuracy, LaTeX tables
│
└── results/                               # Analysis outputs
    ├── section_a_llm_performance.csv
    ├── section_b_intent_distribution.csv
    ├── section_c_signal_loss.csv
    ├── section_d_safety_gate.csv
    ├── latex_tables.tex                   # Paste directly into Overleaf
    └── final_analysis_YYYYMMDD.txt        # Full printed analysis output
```

### JSONL Decision Record Schema

Every line in a `flight_*.jsonl` file is one decision:

```json
{
  "id": "uuid",
  "timestamp": "2026-02-28T03:02:27",
  "flight_time_s": 89.4,
  "input": {
    "battery_pct": 50.0,
    "ekf_confidence": 0.80,
    "current_pos": [12.3, -4.1, -10.0],
    "visited_sectors": ["SECTOR_A"],
    "signal_loss_count": 1,
    "post_signal_loss_reentry": false,
    "last_loss_duration_s": 5.0,
    "pre_loss_intent": "SEARCH_NEXT_PRIORITY",
    "event_log_at_decision": ["[T+89s] Arrived SECTOR_A. Battery: 50%"]
  },
  "llm_output": {
    "intent": 3,
    "intent_name": "SEARCH_NEXT_PRIORITY",
    "reasoning": "Battery healthy. Resume search.",
    "confidence": 90,
    "response_time_seconds": 3.24,
    "fallback_used": false,
    "was_reentry_decision": false
  },
  "edge_metrics": {
    "pi_ip": "10.0.0.119",
    "model": "llama3.2:1b",
    "prompt_tokens": 538,
    "output_tokens": 46,
    "inference_time_s": 13.84,
    "tokens_per_sec": 3.3,
    "network_rtt_s": 51.88
  },
  "outcome": {
    "waypoint_reached": true,
    "battery_at_arrival": 48.0,
    "time_to_arrive_seconds": 10.0,
    "dwell_completed": true
  }
}
```

> `edge_metrics` is populated only in Config B (Pi edge) runs. It is absent in Config A (local) runs.

### Log File Quick Reference

| File | Contains | Used For |
|------|----------|----------|
| `flight_*.jsonl` | Every LLM decision with full input/output/metrics | Primary research data |
| `run_*_local_terminal.log` | Full terminal output — local runs | Safety gate counts, error tracking |
| `run_*_edge_terminal.log` | Full terminal output — edge runs | Timeout events, `[EDGE]` inference lines |
| `pi_hardware_log_*.txt` | CPU / RAM / Temp every 5s from Pi | Paper Section E hardware metrics |
| `results/latex_tables.tex` | Auto-generated LaTeX tables | Paste into Overleaf paper |

### Running the Analysis Script

```bash
# Full analysis — all configs
python3 ~/uav_dataset/analyze_paper_data.py

# Save output to file
python3 ~/uav_dataset/analyze_paper_data.py 2>&1 | \
  tee ~/uav_dataset/results/final_analysis_$(date +%Y%m%d_%H%M%S).txt

# Get LaTeX tables
cat ~/uav_dataset/results/latex_tables.tex
```

<br>

---

## 📈 Research Results Summary

Experimental results across 5 sessions per configuration (60 local, 33 edge decisions):

### LLM Inference Performance

| Metric | Local (LLaMA3 8B) | Pi Edge (LLaMA 3.2 1B) |
|--------|------------------|------------------------|
| Total decisions | 60 | 33 |
| Successful (n) | 59 | 28 |
| Mean latency | **3.241s** | **52.642s** |
| Std deviation | 0.714s | 11.304s |
| Median latency | 3.210s | 49.350s |
| Mean confidence | 82.8% | 90.0% |
| Timeouts | 0 | 4 (12.1%) |
| Fallbacks | 1 | 5 |

### Signal Loss Re-entry Accuracy

| Metric | Local (8B) | Pi Edge (1B) |
|--------|-----------|--------------|
| Total losses | 5 | 5 |
| Re-entry decisions | 10 | 8 |
| Avg loss duration | 5.0s | 33.0s |
| Correct re-entry | **10/10 (100%)** | **6/8 (75%)** |

### Raspberry Pi 5 Hardware (during inference)

| Metric | Value |
|--------|-------|
| Avg CPU | 18.5% |
| Peak CPU | 63.4% |
| Avg RAM | 1,254 MB |
| Peak RAM | 2,328 MB |
| Avg Temperature | 47.4°C |
| **Peak Temperature** | **78.5°C** *(limit: 80°C)* |

> Full results and LaTeX tables in `uav_dataset/results/`. See the accompanying paper for complete analysis.

<br>

---

## 📁 Project File Structure

```
UAV-Triage/
└── ros_ws/
    ├── src/
    │   ├── uav_control/uav_control/
    │   │   ├── autonomous_uav_final.py           # Config A — Local GCS (LLaMA3 8B)
    │   │   ├── autonomous_uav_pi_edge_short.py   # Config B — Pi 5 Edge (LLaMA 3.2 1B)
    │   │   ├── uav_dashboard.py                  # Live operator terminal display
    │   │   └── uav_label_dataset.py              # Post-flight decision labeler
    │   ├── px4_msgs/                             # PX4 ↔ ROS 2 message definitions
    │   ├── px4_offboard/                         # Offboard control package
    │   └── px4_ros_com/                          # ROS 2 bridge utilities
    └── install/                                  # colcon build outputs
```

| File | Role |
|------|------|
| `autonomous_uav_final.py` | **Config A main script** — local LLaMA3 8B |
| `autonomous_uav_pi_edge_short.py` | **Config B main script** — Pi 5 edge LLaMA 3.2 1B |
| `uav_dashboard.py` | **Live display** — run in separate terminal, Q to quit |
| `uav_label_dataset.py` | **Labeling tool** — review decisions, export fine-tuning data |
| `uav_dataset/analyze_paper_data.py` | **Analysis script** — generates all paper tables |

<br>

---

## 🔧 Troubleshooting

**Pi turns solid red LED / crashes during inference:**
> Thermal shutdown — 80°C limit hit. Fix: add passive heatsink, ensure `CPUQuota=200%`
> in override.conf, use official 27W Pi 5 PSU. Wait 60s after power cycle.

**Ollama on Pi not reachable from laptop:**
```bash
# On Pi — verify binding on all interfaces
ss -tlnp | grep 11434   # Must show 0.0.0.0:11434

# From laptop
curl http://<pi-ip>:11434/api/tags
```

**Edge script times out every query:**
> Check Pi temp: `ssh <user>@<pi-ip> "vcgencmd measure_temp"` — if >70°C, wait to cool.
> Check WiFi: `ping <pi-ip>` — must be <5ms for stable operation.
> If prompt tokens >700, prompt is growing too large — restart with fresh run.

**Gazebo doesn't open or freezes:**
```bash
pkill -f gz && pkill -f gzserver && pkill -f gzclient
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```

**`/fmu/` topics not appearing:**
```bash
# DDS agent MUST start before PX4
ros2 topic list | grep fmu   # Must show /fmu/in/ and /fmu/out/
```

**`colcon build` fails on px4_msgs:**
```bash
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select px4_msgs
colcon build --symlink-install
```

**Ollama not responding (local):**
```bash
sudo systemctl restart ollama
curl http://localhost:11434/api/generate \
  -d '{"model":"llama3","prompt":"say READY","stream":false}'
```

**JSON parse failure from Pi:**
> Response truncated mid-JSON — `num_predict` too low.
> Increase to 150 in the edge script options dict.

**Dashboard says "Waiting for UAV system to start":**
```bash
ls /tmp/uav_dashboard_state.json   # Must exist — written by node every 0.5s
```

**T or Y key snaps back immediately:**
> Overrides lock until `R` is pressed. Confirm `[EKF_OVR]` or `[BATT_OVR]` shows
> in the status line — these confirm the lock is active.

<br>

---

## 👥 Contributors

| GitHub | Role |
|--------|------|
| [@nicheroboticslab](https://github.com/nicheroboticslab) | Primary development & ROS 2 integration |
| [@BHUVANRJ](https://github.com/BHUVANRJ) | Autonomous architecture & LLM integration |

> **Institution:** University of Michigan - Dearborn
> **Project:** EGLe — Edge-Deployed LLM Autonomy for UAV Triage
> **Year:** 2026

<br>

---

## 📄 License

Developed for academic research purposes.
University of Michigan - Dearborn © 2026

---

*Stack: ROS 2 Humble · PX4 v1.14 · Gazebo · Micro XRCE-DDS · Ollama · LLaMA3 8B · LLaMA 3.2 1B · Raspberry Pi 5 (16GB) · Python 3.10 · Ubuntu 22.04*
