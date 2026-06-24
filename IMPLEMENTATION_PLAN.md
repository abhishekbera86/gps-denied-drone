# GPS-Denied Autonomous Quadcopter — Implementation Plan

> This is the living technical document for the project.
> Update it as phases are completed and decisions change.

---

## What We're Building

A **GPS-denied autonomous quadcopter** that:
- Flies using **Visual-Inertial Odometry (OpenVINS + RealSense D435i)**
- Runs on **PX4 v1.14 + ROS2 Humble**
- Is **fully containerized** (Docker Compose only — zero host install mess)
- Uses **the same autonomy code** in simulation AND on the real drone
- Is tested phase-by-phase: simulation first → hardware second

---

## Architecture: Sim-to-Real

The core design rule: **autonomy code never knows if it's in sim or real.**

```
┌──────────────────────────────────────────────────────────┐
│                   quad_core (ROS2 package)                │
│           SHARED — identical in sim and on hardware       │
│                                                           │
│   offboard_controller.py   ← arm / takeoff / hover       │
│   mission_planner.py       ← waypoint state machine      │
│   vio_bridge.py            ← OpenVINS → PX4 EKF2         │
└────────────────────┬─────────────────────────────────────┘
                     │  ROS2 topics (identical interface)
            ┌────────┴────────┐
            │                 │
   ┌────────▼───────┐ ┌───────▼────────┐
   │   quad_sim     │ │   quad_real    │
   │  launch files  │ │  launch files  │
   │  + sim config  │ │  + hw config   │
   │                │ │                │
   │  Gazebo SITL   │ │  RealSense     │
   │  PX4 SITL      │ │  Pixhawk UART  │
   └────────────────┘ └────────────────┘
```

**Rule**: Never put flight logic in `quad_sim` or `quad_real`.
Only launch files and hardware config go there.

---

## Data Flow

```
Camera (RealSense D435i / Gazebo camera plugin)
        ↓
OpenVINS (VIO — visual-inertial odometry)
        ↓  /openvins/odometry  (ENU, nav_msgs/Odometry)
vio_bridge.py (ENU→NED conversion + PX4 msg format)
        ↓  /fmu/in/vehicle_visual_odometry
Micro XRCE-DDS Agent
        ↓  UDP 8888
PX4 EKF2 (state estimation — GPS disabled, vision enabled)
        ↓
PX4 Flight Controller (position control loop)
        ↓
ESC / Motors
```

---

## Key Technical Decisions

| Decision | Choice | Reason |
|---|---|---|
| PX4 image | Build from source (v1.14.3) | Pins firmware + px4_msgs to same tag; eliminates #1 silent failure |
| Gazebo version | Garden (gz-garden) | What PX4 v1.14 officially supports |
| Simulation mode | HEADLESS=1 always | No GPU on ThinkPad W541 |
| CPU throttle | PX4_SIM_SPEED_FACTOR=0.5 | Prevents EKF timing failures on slow CPU |
| DDS middleware | CycloneDDS | Most reliable with PX4 on network_mode:host |
| DDS bridge | Micro XRCE-DDS Agent (eProsima) | NOT micro-ros-agent — those are different! |
| VIO engine | OpenVINS | Native ROS2, supports D435i, lightweight |
| Onboard computer | Orange Pi 5 (ARM64) | Phase 5 hardware |
| Companion serial | UART → Pixhawk | /dev/ttyUSB0 or /dev/serial0 |

---

## Version Pins (also in .env)

```
PX4_VERSION          = v1.14.3
PX4_MSGS_TAG         = release/1.14
ROS_DISTRO           = humble
REALSENSE_SDK_VERSION = v2.55.1
OPENVINS_BRANCH      = master
```

---

## Project File Tree

```
px4_docker_ws/
│
├── IMPLEMENTATION_PLAN.md        ← this file
├── README.md                     ← installation guide
├── .env                          ← pinned versions + shared vars
├── .gitignore
├── Makefile                      ← convenience commands
├── docker-compose.yml            ← all services (profiles: sim, vio, hw)
│
├── docker/
│   ├── Dockerfile.px4            ← PX4 SITL + Gazebo Garden (x86_64)
│   ├── Dockerfile.ros2           ← ROS2 Humble + px4_msgs + uXRCE-DDS
│   ├── Dockerfile.vio            ← OpenVINS (Phase 3+)
│   └── Dockerfile.hw             ← RealSense ARM64 for Orange Pi (Phase 5)
│
├── ros2_ws/src/
│   ├── quad_core/                ← AUTONOMY CODE (sim-to-real shared)
│   │   └── quad_core/
│   │       ├── offboard_controller.py
│   │       ├── mission_planner.py
│   │       └── vio_bridge.py
│   ├── quad_sim/                 ← SIMULATION ONLY (launch + config)
│   │   ├── launch/
│   │   └── config/
│   └── quad_real/                ← HARDWARE ONLY (launch + config)
│       ├── launch/
│       └── config/
│
├── config/
│   ├── px4_params/
│   │   ├── sim_gps_denied.params
│   │   └── real_gps_denied.params
│   ├── openvins/
│   │   ├── d435i_sim.yaml
│   │   └── d435i_real.yaml
│   └── uxrce_dds_topics.yaml
│
└── scripts/
    ├── setup_host.sh
    ├── health_check.sh
    └── px4_params_upload.sh
```

---

## Phase Breakdown

### Phase 0 — Foundation ✅
**Goal**: Repo initialized, Docker images build, containers start.
**Gate**: `make sim-up` → 3 containers healthy.

### Phase 1 — PX4 ↔ ROS2 Communication
**Goal**: PX4 SITL talks to ROS2. Flight data topics visible.
**Gate**: `ros2 topic list` shows `/fmu/out/vehicle_odometry`.

### Phase 2 — Offboard Hover
**Goal**: ROS2 node arms drone, climbs to 5m, hovers stably.
**Gate**: `ros2 launch quad_sim sim_offboard.launch.py` → z ≈ -5.0

### Phase 3 — Waypoint Mission (GPS)
**Goal**: Drone flies autonomous 4-point square and lands.
**Gate**: Mission completes, `DISARMED` state reached.

### Phase 4 — GPS-Denied VIO in Simulation
**Goal**: GPS disabled, drone flies purely on VIO odometry.
**Gate**: Waypoint mission completes with GPS off, EKF healthy.

### Phase 5 — Real Hardware (Orange Pi + Pixhawk)
**Goal**: Same autonomy code on real drone, GPS-denied flight.
**Gate**: Real drone hovers at 1m, stable, using RealSense VIO.

---

## Sim-to-Real Transfer Table

| Component | Simulation | Real Hardware | Identical? |
|---|---|---|---|
| offboard_controller.py | ✅ | ✅ | **YES** |
| mission_planner.py | ✅ | ✅ | **YES** |
| vio_bridge.py | ✅ | ✅ | **YES** |
| PX4 topic interface | via DDS/UDP | via DDS/UART | **YES** |
| Launch files | quad_sim/launch/ | quad_real/launch/ | No |
| EKF2 params | sim_gps_denied | real_gps_denied | Slightly different |
| Camera driver | Gazebo plugin | realsense2_camera | No |
| OpenVINS config | d435i_sim.yaml | d435i_real.yaml | No |

---

## Git Tag Strategy

```
v0.1.0  → Phase 0: repo + Docker builds
v0.2.0  → Phase 1: PX4↔ROS2 communication
v0.3.0  → Phase 2: offboard hover working
v0.4.0  → Phase 3: waypoint mission working
v0.5.0  → Phase 4: GPS-denied sim working
v1.0.0  → Phase 5: real hardware flying GPS-denied
```
