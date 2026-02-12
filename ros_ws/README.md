# 🚁 UAV Autonomous Triage System

> An intelligent UAV system combining **PX4 autopilot**, **ROS 2 Humble**, and a **local LLM (LLaMA3 via Ollama)** for autonomous mission decision-making, battery-aware navigation, and EKF health monitoring — simulated in **Gazebo** and monitored via **QGroundControl**.

<br>

## 📋 Table of Contents

- [Project Overview](#-project-overview)
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
- [Project Structure](#-project-structure)
- [Troubleshooting](#-troubleshooting)
- [Contributors](#-contributors)

<br>

---

## 🧠 Project Overview

This project implements a **fully autonomous UAV triage architecture** where the drone:

- Flies manually via keyboard in real-time at 20 Hz
- Switches to **autonomous mode** after 15 seconds of no keyboard input
- Uses a **local LLM (LLaMA3)** to decide between 7 mission intents based on battery level, EKF confidence, and mission history
- Executes **battery-aware waypoint navigation** with dynamic altitude capping
- Loiters over targets in circular orbits (r=15m, ω=0.3 rad/s)
- Returns to base automatically when battery is critical

All simulation runs **100% locally** using PX4 SITL + Gazebo with no cloud dependencies.

<br>

---

## 🏗 System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     SENSORS / INPUTS                    │
│         Keyboard · Battery · EKF · Position             │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────┐
│               COMM LOSS DETECTOR                        │
│   Scores 3 signals → triggers AUTONOMOUS if score ≥ 1  │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────┐
│                  LLM DECISION ENGINE                    │
│        Ollama / LLaMA3  →  picks 1 of 7 intents        │
│          Runs in background thread every 30s            │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────┐
│               WAYPOINT GENERATOR                        │
│      Converts intent → battery-aware [x, y, z]         │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────┐
│              PX4 SITL + GAZEBO                          │
│    /fmu/in/trajectory_setpoint  @  20 Hz                │
└─────────────────────────────────────────────────────────┘
         ↑                                    │
         └──── Feedback: pos, battery, EKF ───┘
```

### 7 Mission Intents (LLM Output)

| # | Intent | Trigger Condition | Behavior |
|---|--------|-------------------|----------|
| 1 | `HOLD_SAFE` | EKF confidence < 0.3 | Loiter circle r=15m, ω=0.3 rad/s |
| 2 | `CONTINUE_ASSIGNED_TASK` | Resume after interruption | Fly to assigned sector |
| 3 | `SEARCH_NEXT_PRIORITY` | Default / all clear | Fly to highest-priority unvisited sector |
| 4 | `INSPECT_TARGET_REPORT` | Unconfirmed target nearby | Confirm nearest victim target |
| 5 | `COMMS_REACQUIRE` | Comms degraded | Fly to [30, 30, -25] comms point |
| 6 | `RETURN_TO_BASE` | Battery < LOW threshold | RTB to [0, 0, -5] |
| 7 | `ABORT_AND_SAFE` | Battery < CRITICAL threshold | Emergency land immediately |

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
> The LLM handles all autonomous mission decisions inside `autonomous_uav.py`.
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
# Downloads LLaMA3 8B model (~4.7 GB)
# Make sure you have at least 6 GB free disk space
ollama pull llama3
```

**Confirm the model downloaded successfully:**
```bash
ollama list
# Expected:
# NAME              ID              SIZE      MODIFIED
# llama3:latest     365c0bd3c000    4.7 GB    X seconds ago
```

**Test the model responds correctly:**
```bash
ollama run llama3 "Reply with only the number 3. Nothing else."
# Expected output: 3
```

> ⚠️ **Ollama must be running before you start the control script.**
> It starts automatically as a service after install. If it ever stops:
> ```bash
> ollama serve
> # Or restart the service:
> sudo systemctl restart ollama
> ```
> The control script calls Ollama at `http://localhost:11434` — ensure this port is free.

---

### 6. Python Dependencies

```bash
pip3 install pynput requests
```

| Package | Purpose |
|---------|---------|
| `pynput` | Captures live keyboard input for manual drone control |
| `requests` | HTTP calls to Ollama REST API at `localhost:11434` |
| `rclpy` | ROS 2 Python client — **included with ROS 2 Humble**, no install needed |

**Verify:**
```bash
python3 -c "import pynput, requests; print('✅ All packages OK')"
```

---

### 7. Build the ROS 2 Workspace

```bash
cd ~/ros_ws

# Auto-install all declared ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build --symlink-install

# Source the workspace (do this in every new terminal)
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

**Verify all packages built:**
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

### Terminal 3 — ROS 2 Control Script

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
python3 src/px4_offboard/px4_offboard/offboard_fly.py
```

> Drone arms and takes off automatically.
> Fly manually with keyboard arrows.
> After **15s of no input** → LLM (`autonomous_uav.py`) takes autonomous control.

---

### Terminal 4 — QGroundControl

```bash
cd ~
./QGroundControl-x86_64.AppImage
```

> Auto-connects to simulation. Monitor battery, GPS, flight mode, and mission status.

---

### ✅ Quick-Start (Copy-Paste)

```bash
# Terminal 1 — DDS Bridge
cd ~/Micro-XRCE-DDS-Agent/build && MicroXRCEAgent udp4 -p 8888

# Terminal 2 — PX4 + Gazebo
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 3 — Control Script
cd ~/ros_ws && source /opt/ros/humble/setup.bash && source install/local_setup.bash && python3 src/px4_offboard/px4_offboard/offboard_fly.py

# Terminal 4 — QGroundControl
~/QGroundControl-x86_64.AppImage
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

| Key | Action | Result |
|-----|--------|--------|
| `T` | Force bad EKF (0.2) | LLM picks intent `1` → HOLD_SAFE circular loiter |
| `Y` | Force low battery (10%) | LLM picks intent `6` → RETURN_TO_BASE |
| `R` | Reset to normal | Restores EKF=1.0, battery=100% |
| `F` | Force LLM re-query now | Skips 30s cooldown, immediate decision |
| `1`–`7` | Force specific intent | Bypasses LLM entirely, directly activates |
| `H` | Show help | Prints all available keys in terminal |

<br>

---

## 📁 Project Structure

```
ros_ws/
└── src/
    ├── px4_msgs/                        # PX4 ↔ ROS 2 message definitions
    │   ├── msg/                         # All PX4 uORB message types (.msg)
    │   │   ├── VehicleLocalPosition.msg #   Drone position (x, y, z)
    │   │   ├── VehicleCommand.msg       #   Arm / Land / Mode commands
    │   │   ├── TrajectorySetpoint.msg   #   Target position setpoints
    │   │   ├── BatteryStatus.msg        #   Battery voltage & percentage
    │   │   ├── EstimatorStatusFlags.msg #   EKF health flags
    │   │   └── ...                      #   200+ PX4 message definitions
    │   ├── srv/
    │   │   └── VehicleCommand.srv
    │   └── package.xml
    │
    ├── px4_offboard/                    # ✈️ Main offboard control package
    │   └── px4_offboard/
    │       ├── __init__.py
    │       ├── offboard_fly.py          # 🔑 PRIMARY ENTRY POINT — run this
    │       └── offboard_control_fixed.py
    │
    ├── px4_ros_com/                     # PX4 ↔ ROS 2 bridge utilities
    │   ├── src/
    │   │   ├── examples/
    │   │   │   ├── offboard/
    │   │   │   │   ├── offboard_control.cpp
    │   │   │   │   └── offboard_control_srv.cpp
    │   │   │   └── listeners/
    │   │   │       ├── sensor_combined_listener.cpp
    │   │   │       └── vehicle_gps_position_listener.cpp
    │   │   └── lib/
    │   │       └── frame_transforms.cpp # NED ↔ ENU coordinate transforms
    │   ├── include/px4_ros_com/
    │   │   └── frame_transforms.h
    │   ├── launch/
    │   │   ├── offboard_control_launch.yaml
    │   │   └── sensor_combined_listener.launch.py
    │   └── scripts/
    │       ├── setup_system.bash
    │       └── build_ros2_workspace.bash
    │
    └── uav_control/                     # 🤖 Autonomous decision package
        └── uav_control/
            ├── __init__.py
            └── autonomous_uav.py        # LLM engine + state machine + dwell timer
```

### Key Files at a Glance

| File | Role |
|------|------|
| `px4_offboard/offboard_fly.py` | **Main script** — keyboard control, comm loss detection, 20Hz publisher |
| `uav_control/autonomous_uav.py` | **LLM brain** — queries Ollama, runs state machine, generates waypoints |
| `px4_msgs/msg/` | Message definitions for all PX4 ↔ ROS 2 topics |
| `px4_ros_com/src/lib/frame_transforms.cpp` | NED ↔ ENU coordinate frame conversion utilities |

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
# Check service
systemctl status ollama

# Restart
sudo systemctl restart ollama

# Test API directly
curl http://localhost:11434/api/generate \
  -d '{"model":"llama3","prompt":"say 3","stream":false}'
```

**Script exits immediately after sourcing:**
```bash
# Make sure BOTH source commands run every new terminal
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/local_setup.bash
python3 src/px4_offboard/px4_offboard/offboard_fly.py
```

**QGC can't find drone:**
> PX4 SITL must be running first. QGC auto-connects to `localhost:14550` UDP. No manual config needed for SITL.

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
