# 🚁 GPS-Denied Autonomous Quadcopter Stack

> **PX4 v1.14.3 + ROS 2 Humble + Aerostack2 + OpenVINS VIO — Fully Containerised**

A fully Docker-based autonomous quadcopter stack that flies **without GPS** using Visual-Inertial Odometry (OpenVINS + RealSense D435i). Built on the **Aerostack2** framework — zero compilation required on the host machine. Designed for simulation-first development with a clean Sim-to-Real path to physical hardware (Orange Pi 5 + RealSense D435i + Pixhawk).

**GitHub**: [github.com/abhishekbera86/gps-denied-drone](https://github.com/abhishekbera86/gps-denied-drone)

---

## Table of Contents

1. [Architecture](#architecture)
2. [Prerequisites](#prerequisites)
3. [Quick Start](#quick-start)
4. [Project Structure](#project-structure)
5. [Phase Guide](#phase-guide)
   - [Phase 0 — Foundation](#phase-0--foundation-current)
   - [Phase 1 — PX4 ↔ ROS2 Communication](#phase-1--px4--ros2-communication)
   - [Phase 2 — Offboard Hover](#phase-2--offboard-hover)
   - [Phase 3 — Waypoint Mission](#phase-3--waypoint-mission)
   - [Phase 4 — GPS-Denied VIO in Simulation](#phase-4--gps-denied-vio-in-simulation)
   - [Phase 5 — Real Hardware Deployment](#phase-5--real-hardware-orange-pi--pixhawk)
6. [Docker Services](#docker-services)
7. [Makefile Commands](#makefile-commands)
8. [Configuration Reference](#configuration-reference)
9. [Hardware Setup (Orange Pi)](#hardware-setup-orange-pi)
10. [Calibration Guide](#calibration-guide)
11. [Troubleshooting](#troubleshooting)
12. [Contributing](#contributing)

---

## Architecture

### Sim-to-Real Design

The most important design principle: **autonomy code never knows if it's in simulation or on real hardware.**

```
┌──────────────────────────────────────────────────────────────┐
│                   quad_core  (ROS2 package)                   │
│          ← Same Python code in sim AND on real drone →        │
│                                                               │
│   offboard_controller.py   arm / takeoff / hover             │
│   mission_planner.py       waypoint state machine            │
│   vio_bridge.py            OpenVINS → PX4 EKF2 bridge        │
└──────────────────┬────────────────────────────────────────────┘
                   │  ROS2 DDS topics (identical interface)
          ┌────────┴────────┐
          │                 │
 ┌────────▼───────┐ ┌───────▼────────┐
 │   quad_sim     │ │   quad_real    │
 │  launch files  │ │  launch files  │
 │  + sim config  │ │  + hw config   │
 │                │ │                │
 │  Gazebo SITL   │ │  RealSense D435│
 │  PX4 SITL      │ │  Pixhawk UART  │
 └────────────────┘ └────────────────┘
```

### Data Flow

```
Camera (RealSense D435i / Gazebo camera plugin)
        │
        ▼
OpenVINS (Visual-Inertial Odometry)
        │  /openvins/odometry  [ENU, nav_msgs/Odometry]
        ▼
vio_bridge.py  (ENU → NED frame conversion)
        │  /fmu/in/vehicle_visual_odometry  [NED, px4_msgs]
        ▼
Micro XRCE-DDS Agent  (UDP port 8888)
        │
        ▼
PX4 EKF2  (state estimation — GPS disabled, vision enabled)
        │
        ▼
PX4 Position Controller → ESCs → Motors
```

### Version Pins

| Component | Version | Why pinned |
|---|---|---|
| PX4 Autopilot | v1.14.3 | Stable, well-documented |
| Aerostack2 | nightly-humble | Full autonomy framework — pre-built |
| ROS 2 | Humble (LTS) | Long-term support until 2027 |
| DDS | CycloneDDS | Most reliable with PX4 on host network |
| VIO | OpenVINS (apt) | Native ROS 2, supports D435i |
| RealSense SDK | **v2.58.2** | Required for firmware 5.17.3.10 |

---

## Prerequisites

### Host Machine (Development / Simulation)
- **OS**: Ubuntu 22.04 LTS
- **CPU**: x86_64, 4+ cores (ThinkPad W541 or better)
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 30GB free (Docker images + PX4 build ≈ 20GB)
- **GPU**: **NOT required** — runs headless (HEADLESS=1)
- **Software**: Docker Engine + docker compose plugin (installed by `setup_host.sh`)

### Orange Pi (Real Hardware — Phase 5)
- **Board**: Orange Pi 5 (Rockchip RK3588, ARM64)
- **OS**: Ubuntu 22.04 for Orange Pi
- **RAM**: 8GB or 16GB version recommended
- **Storage**: 64GB+ microSD or eMMC
- **Connections**: USB 3.0 for RealSense D435i, UART for Pixhawk

### Real Drone Hardware (Phase 5)
- Pixhawk (any modern variant: 4, 6C, 6X, Cube Orange)
- Intel RealSense D435i
- Quadcopter frame (any, configured as x500 or similar)
- Battery, ESCs, motors

---

## Quick Start

### 4 Commands to Fly in Simulation (no GPU required)

```bash
# 1. One-time host setup (installs Docker, configures groups)
bash scripts/setup_host.sh

# 2. Build Docker images (~5-10 min first time — no compilation on host!)
make build

# 3. Start simulation stack (PX4 SITL headless + Aerostack2 + DDS Agent)
make sim-up

# 4. Wait ~60s for PX4 to boot, then launch Aerostack2 nodes
make health       # all green?
make as2-launch   # opens tmux with all AS2 nodes
```

Then run the autonomous GPS-denied mission:

```bash
# Uploads EKF2 VIO parameters, then runs the square mission
make params-upload
make mission
```

---

## Project Structure

```
px4_docker_ws/
│
├── IMPLEMENTATION_PLAN.md     ← Technical design document
├── README.md                  ← This file
├── .env                       ← Version pins + runtime settings
├── .gitignore
├── Makefile                   ← All convenience commands
├── docker-compose.yml         ← Service definitions (profiles: sim, vio, hw)
│
├── docker/
│   ├── Dockerfile.px4_sitl    ← px4io base + PX4 v1.14.3 SITL baked in (3-5 min)
│   ├── Dockerfile.as2         ← aerostack2/nightly-humble + apt AS2 packages
│   ├── Dockerfile.vio         ← ros:humble + apt OpenVINS
│   └── Dockerfile.hw          ← ARM64: librealsense v2.58.2 + AS2 + OpenVINS
│
├── ros2_ws/src/
│   ├── quad_core/             ← SHARED AUTONOMY (identical in sim AND real)
│   │   ├── mission.py                   ← Aerostack2 Python API GPS-denied mission
│   │   └── config/
│   │       ├── ekf2_vio.params          ← PX4 EKF2 GPS-denied parameters
│   │       └── vio_d435i.yaml           ← OpenVINS D435i calibration
│   │
│   ├── quad_sim/              ← SIMULATION ONLY (launch files + YAML configs)
│   │   ├── launch_as2.bash              ← tmux-based AS2 node launcher
│   │   └── config/
│   │       ├── world.yaml               ← Gazebo world + drone spawn
│   │       ├── as2_platform_sim.yaml    ← UDP:8888 to PX4 SITL
│   │       ├── state_estimator_sim.yaml ← raw_odometry plugin
│   │       └── motion_controller_sim.yaml
│   │
│   └── quad_real/             ← HARDWARE ONLY (launch files + serial configs)
│       ├── launch_as2.bash              ← hardware node launcher
│       └── config/
│           ├── as2_platform_hw.yaml     ← serial /dev/ttyUSB0 @ 921600
│           ├── state_estimator_hw.yaml
│           └── realsense_hw.yaml        ← D435i camera parameters
│
├── config/
│   ├── px4_params/
│   │   ├── sim_gps_denied.params        ← EKF2 VIO params (sim)
│   │   └── real_gps_denied.params       ← EKF2 VIO params (real)
│   ├── openvins/
│   │   ├── d435i_sim.yaml               ← OpenVINS sim config
│   │   └── d435i_real.yaml              ← OpenVINS real config (calibrate!)
│   └── uxrce_dds_topics.yaml            ← Bridged topic list
│
└── scripts/
    ├── setup_host.sh          ← One-shot host prerequisite installer
    ├── health_check.sh        ← Verify stack is working
    └── px4_params_upload.sh   ← Upload .params to PX4
```

---

## Phase Guide

### Phase 0 — Foundation (CURRENT)

**Goal**: Repository initialized. Docker images build successfully. Containers start.

**Verification**:
```bash
make build        # All images build without errors
make sim-up       # 3 containers running: px4, ros2
make px4-shell    # Enter px4 container, see /PX4-Autopilot directory
make ros2-shell   # Enter ros2 container, ROS2 sources correctly
make sim-down     # Clean shutdown
```

**Git tag**: `v0.1.0`

---

### Phase 1 — PX4 ↔ ROS2 Communication

**Goal**: Start PX4 SITL and verify ROS2 can see flight data topics.

**Steps**:
```bash
# Terminal 1: Start containers
make sim-up

# Terminal 2: Start PX4 SITL (inside container)
make px4-start
# Wait ~30 seconds for "Ready to fly" in PX4 logs

# Terminal 3: Check ROS2 topics
make ros2-shell
# Inside container:
ros2 topic list
```

**Expected topics**:
```
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status
/fmu/out/vehicle_local_position
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_visual_odometry
```

**Verification gate**: All topics above are visible ✓

**Git tag**: `v0.2.0`

---

### Phase 2 — Offboard Hover

**Goal**: A ROS2 node arms the drone and flies it to 5m altitude.

**Steps**:
```bash
# Start everything
make sim-up && make px4-start
# Wait 30s for SITL to initialize

# Launch offboard controller
make ros2-shell
ros2 launch quad_sim sim_offboard.launch.py
```

**Expected behavior**:
1. Node publishes `OffboardControlMode` at 10Hz (keepalive)
2. After 1 second: sends `VehicleCommand` to ARM
3. After arming: sends `VehicleCommand` to switch to OFFBOARD mode
4. Drone climbs to 5m and holds position
5. Press Ctrl-C → drone lands and disarms

**Verification gate**: `ros2 topic echo /fmu/out/vehicle_local_position` shows `z ≈ -5.0` ✓

**Git tag**: `v0.3.0`

---

### Phase 3 — Waypoint Mission

**Goal**: Drone executes a 4-point square mission and returns to land.

**Steps**:
```bash
make ros2-shell
ros2 launch quad_sim sim_offboard.launch.py mission:=true
```

**State machine**:
```
IDLE → ARM → TAKEOFF (5m) → WP[0] → WP[1] → WP[2] → WP[3] → LAND → DISARM
```

**Verification gate**: Mission completes, `DISARMED` state reached ✓

**Git tag**: `v0.4.0`

---

### Phase 4 — GPS-Denied VIO in Simulation

**Goal**: GPS is disabled. Drone flies purely on Visual-Inertial Odometry.

**Steps**:

1. Upload GPS-denied parameters to PX4:
   ```bash
   bash scripts/px4_params_upload.sh config/px4_params/sim_gps_denied.params sim
   ```

2. Start VIO stack:
   ```bash
   make vio-up
   ```

3. Launch VIO mission:
   ```bash
   make ros2-shell
   ros2 launch quad_sim sim_vio.launch.py
   ```

**Verification**:
```bash
# VIO data flowing to PX4:
ros2 topic echo /fmu/in/vehicle_visual_odometry

# PX4 EKF using vision (not GPS):
ros2 topic echo /fmu/out/vehicle_status
# Look for: gps_failure=true, ev_valid=true
```

**Verification gate**: Waypoint mission completes with GPS disabled, EKF healthy ✓

**Git tag**: `v0.5.0`

---

### Phase 5 — Real Hardware (Orange Pi + Pixhawk)

**Goal**: The same autonomy code runs on real hardware. GPS-denied hover achieved.

#### Orange Pi Setup

1. **Flash Ubuntu 22.04** for Orange Pi 5 to eMMC/SD
2. **Install Docker** on Orange Pi:
   ```bash
   curl -fsSL https://get.docker.com | bash
   sudo usermod -aG docker $USER
   ```

3. **Clone repo** on Orange Pi:
   ```bash
   git clone https://github.com/YOUR_USERNAME/YOUR_REPO.git
   cd YOUR_REPO
   ```

4. **Build hardware image** (build on Orange Pi directly, ~2hrs):
   ```bash
   docker build -f docker/Dockerfile.hw -t quad/hw .
   ```
   Or cross-compile on dev machine (faster):
   ```bash
   docker buildx build --platform linux/arm64 \
     -f docker/Dockerfile.hw -t quad/hw --load .
   # Transfer image to Orange Pi
   docker save quad/hw | ssh orangepi 'docker load'
   ```

5. **Connect hardware**:
   - RealSense D435i → USB 3.0 on Orange Pi
   - Pixhawk TELEM2 → UART on Orange Pi (`/dev/ttyUSB0` or `/dev/serial0`)

6. **Upload PX4 parameters** to Pixhawk:
   - QGroundControl → Vehicle Setup → Parameters → Tools → Load from file
   - Select: `config/px4_params/real_gps_denied.params`
   - **Reboot Pixhawk**

7. **Start hardware stack**:
   ```bash
   make hw-up
   ```

8. **Verify RealSense**:
   ```bash
   make ros2-shell
   ros2 topic list | grep camera
   ```

9. **First flight** (hover only, indoor, 1.5m):
   ```bash
   ros2 launch quad_real real_vio.launch.py
   ```

**Verification gate**: Drone hovers at 1.5m, stable, using RealSense VIO ✓

**Git tag**: `v1.0.0`

---

## Docker Services

| Service | Profile | Base Image | Description |
|---|---|---|---|
| `px4_sitl` | `sim` | `px4io/px4-dev-simulation-jammy` | PX4 v1.14.3 + Gazebo Garden (headless). SITL baked in. |
| `aerostack2` | `sim` | `aerostack2/nightly-humble` | DDS Agent + all AS2 nodes via apt. No build needed. |
| `openvins` | `vio` | `ros:humble-ros-base` | OpenVINS VIO — apt installed. |
| `hw_stack` | `hw` | `ros:humble-ros-base` (ARM64) | librealsense v2.58.2 + AS2 + OpenVINS. Orange Pi. |

**Networking**: All services use `network_mode: host`. This is mandatory for DDS multicast discovery to work between containers and the host.

**Volume mount**: `./ros2_ws` is mounted live into the `ros2` container. Edit code on your host machine, build and run inside the container. No rebuild needed when changing Python files.

---

## Makefile Commands

```bash
# Setup
make setup          # Install Docker + configure host (run once)
make build          # Build all Docker images
make build-px4      # Build only PX4 image
make build-ros2     # Build only ROS2 image

# Simulation (Phase 1-4)
make sim-up         # Start simulation stack (px4 + ros2)
make vio-up         # Start sim + OpenVINS
make sim-down       # Stop all containers
make px4-start      # Launch PX4 SITL inside container (HEADLESS)

# Shells
make px4-shell      # bash inside px4 container
make ros2-shell     # bash inside ros2 container

# Hardware (Phase 5 — run on Orange Pi)
make hw-up          # Start hardware stack

# Monitoring
make health         # Full health check (containers + topics)
make logs           # Tail all container logs
make ps             # Container status

# Git
make git-init       # Initialize repo + first commit + v0.1.0 tag

# Cleanup
make clean          # Remove containers + local images
make clean-all      # Remove everything including volumes
```

---

## Configuration Reference

### .env File

Edit `.env` to change versions or settings:

```bash
PX4_VERSION=v1.14.3         # PX4 firmware version
PX4_MSGS_TAG=release/1.14   # Must match PX4_VERSION
PX4_HEADLESS=1              # 1=no Gazebo window (required without GPU)
PX4_SIM_SPEED_FACTOR=0.5    # 0.5=half speed (safe for CPU-only machines)
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # DDS middleware (don't change)
```

> ⚠️ **Always update both `PX4_VERSION` and `PX4_MSGS_TAG` together.**
> A mismatch causes silent DDS communication failure — the hardest bug to debug.

### PX4 EKF2 Parameters (GPS-Denied)

Key parameters in `config/px4_params/sim_gps_denied.params`:

| Parameter | Value | Meaning |
|---|---|---|
| `EKF2_AID_MASK` | 24 | Fuse vision position (bit 3) + vision yaw (bit 4) |
| `EKF2_HGT_MODE` | 3 | Use vision for height (not barometer) |
| `EKF2_EV_DELAY` | 50 | VIO latency compensation (ms) — tune per system |
| `EKF2_EV_NOISE_MD` | 0 | Use covariance from VIO message |
| `SYS_HAS_GPS` | 0 | Tell PX4 there is no GPS |
| `COM_RCL_EXCEPT` | 4 | Allow offboard without RC transmitter |

---

## Hardware Setup (Orange Pi)

### Physical Connections

```
Orange Pi 5
├── USB 3.0 Port 1   → RealSense D435i (USB 3 cable, max 1m)
├── UART (/dev/ttyUSB0 or /dev/serial0) → Pixhawk TELEM2
└── Power            → 5V 3A (use powered USB hub for RealSense)
```

> ⚠️ **Power warning**: The RealSense D435i draws up to 900mA on USB. Use a powered USB 3.0 hub if your Orange Pi doesn't provide enough current. Insufficient power causes random disconnections.

### Pixhawk UART Wiring (TELEM2 → Orange Pi UART)

| Pixhawk TELEM2 | Orange Pi UART | Notes |
|---|---|---|
| TX (pin 3) | RX | Cross-wired |
| RX (pin 4) | TX | Cross-wired |
| GND (pin 6) | GND | Common ground |
| VCC (pin 1) | — | Do NOT connect (different voltage levels) |

### Pixhawk PX4 Parameters for Serial

```
MAV_1_CONFIG = TELEM 2
MAV_1_MODE   = Onboard
MAV_1_RATE   = 0 (auto)
SER_TEL2_BAUD = 921600
UXRCE_DDS_CFG = TELEM 2
```

---

## Calibration Guide

### Camera Intrinsics (Kalibr)

Before real flights, calibrate your specific RealSense D435i camera:

1. **Print calibration target**:
   ```bash
   # Download April grid target from Kalibr
   wget https://github.com/ethz-asl/kalibr/wiki/downloads/kalibr_april_grid.pdf
   ```

2. **Record calibration bag**:
   ```bash
   ros2 bag record \
     /camera/infra1/image_rect_raw \
     /camera/imu \
     -o calibration_bag
   ```

3. **Run Kalibr** (in Kalibr Docker):
   ```bash
   kalibr_calibrate_cameras \
     --bag calibration_bag.db3 \
     --topics /camera/infra1/image_rect_raw \
     --models pinhole-radtan \
     --target kalibr_april_grid.yaml
   ```

4. **Update config**: Replace values in `config/openvins/d435i_real.yaml`.

### IMU Noise Parameters

Run `imu_utils` for ~2 hours with the camera stationary:
```bash
ros2 run imu_utils imu_an -s /camera/imu -b 400
```
Then update `gyroscope_noise_density`, `accelerometer_noise_density` etc. in `d435i_real.yaml`.

---

## Troubleshooting

### ROS2 topics not visible after PX4 SITL starts

**Symptom**: `ros2 topic list` doesn't show `/fmu/out/*` topics.

**Causes & fixes**:
1. `MicroXRCEAgent` not running:
   ```bash
   docker exec ros2 bash -c "pgrep MicroXRCEAgent"
   # If empty: docker restart ros2
   ```
2. PX4 SITL not fully started yet — wait 30s for "Ready to fly"
3. Wrong `px4_msgs` version:
   ```bash
   docker exec ros2 bash -c "cat /px4_msgs_ws/src/px4_msgs/.git/HEAD"
   # Should show release/1.14
   ```

### Simulation is very slow (RTF < 0.5)

**Fix**: Lower simulation speed further:
```bash
# In .env:
PX4_SIM_SPEED_FACTOR=0.25
# Then restart: make sim-down && make sim-up && make px4-start
```

Also close all other applications to free CPU.

### EKF2 diverges / drone flies away in GPS-denied mode

**Causes**:
1. `EKF2_EV_DELAY` too low — increase by 25ms increments
2. VIO frame not correctly converted (ENU vs NED) — check `vio_bridge.py`
3. IMU noise parameters wrong — recalibrate

### Docker build fails (out of disk space)

```bash
# Clean dangling Docker objects
docker system prune -f
# Check space
df -h
# Need 30GB free minimum
```

### RealSense D435i not detected on Orange Pi

```bash
# Check USB connection
lsusb | grep Intel
# Should show: Bus ... Device ...: ID 8086:0b3a Intel Corp. ...

# Check if container can see it
docker exec realsense lsusb | grep Intel

# If not: ensure container is using --privileged and /dev is mounted
```

---

## Contributing

### Git Workflow

```bash
# Initialize repo (first time only)
make git-init

# Feature branch workflow
git checkout -b phase-1/px4-ros2-comms
# ... make changes, test ...
git add .
git commit -m "feat(phase-1): verify PX4↔ROS2 DDS bridge"
git push origin phase-1/px4-ros2-comms

# Tag completed phases
git tag v0.2.0 -m "Phase 1 complete: PX4↔ROS2 communication verified"
git push origin --tags
```

### Commit Message Convention

```
feat(phase-N): short description    ← new feature
fix(phase-N): what was broken       ← bug fix
docs: update README for phase N     ← documentation
config: tune EKF2_EV_DELAY to 75ms  ← config change
```

### Push to GitHub

```bash
# Remote is already configured:
git remote -v
# origin  https://github.com/abhishekbera86/gps-denied-drone.git

# Push all branches and tags
git push -u origin master --tags
```

---

## License

Apache-2.0 — see LICENSE file.
