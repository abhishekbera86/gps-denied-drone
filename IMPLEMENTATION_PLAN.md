# GPS-Denied Autonomous Drone — Implementation Plan (v2 — Aerostack2)

> **Strategy:** Pre-built containers + Aerostack2 framework.
> Keeps `quad_core / quad_sim / quad_real` structure.
> Zero compilation on host. Zero host software installation.

---

## What We're Building

A **GPS-denied autonomous quadcopter** that:
- Flies using **Visual-Inertial Odometry (OpenVINS + RealSense D435i firmware 5.17.3.10)**
- Runs on **PX4 v1.14.3 + ROS 2 Humble + Aerostack2**
- Is **fully containerised** (Docker Compose only — zero host install)
- Uses **the same autonomy code** in simulation AND on the real drone
- Tested phase-by-phase: headless CPU SITL first → hardware second

---

## Architecture: Sim-to-Real

```
┌──────────────────────────────────────────────────────────┐
│                   quad_core  (ROS 2 package)              │
│        SHARED — identical in sim and on hardware         │
│                                                           │
│   mission.py            Aerostack2 Python API mission    │
│   config/ekf2_vio.params    PX4 EKF2 GPS-denied params  │
│   config/vio_d435i.yaml     OpenVINS D435i calibration  │
└────────────────────┬─────────────────────────────────────┘
                     │  identical ROS 2 topic interface
            ┌────────┴────────┐
            │                 │
   ┌────────▼───────┐ ┌───────▼────────┐
   │   quad_sim     │ │   quad_real    │
   │  SIMULATION    │ │  HARDWARE      │
   │                │ │                │
   │  world.yaml    │ │  realsense_hw  │
   │  as2_platform  │ │  as2_platform  │
   │  (UDP:8888)    │ │  (serial UART) │
   │                │ │                │
   │  Gazebo Garden │ │  RealSense D435i│
   │  PX4 SITL      │ │  Pixhawk UART  │
   └────────────────┘ └────────────────┘
```

**Rule:** All flight logic stays in `quad_core`. `quad_sim` and `quad_real`
only contain launch files and YAML config.

---

## Sim-to-Real Switch

The **only** difference between sim and real is one config value:

| | Simulation (quad_sim) | Hardware (quad_real) |
|---|---|---|
| `transport` | `udp` | `serial` |
| `ip / device` | `127.0.0.1` | `/dev/ttyUSB0` |
| `port / baudrate` | `8888` | `921600` |
| `mission.py` | **Identical** | **Identical** |
| `ekf2_vio.params` | **Identical** | **Identical** |
| `vio_d435i.yaml` | Simulated cam | Physical D435i |

---

## Hardware Constraints

| Item | Value |
|---|---|
| Host GPU | None → Gazebo runs **headless** (`PX4_HEADLESS=1`) |
| RealSense firmware | 5.17.3.10 → librealsense SDK **v2.58.2** |
| Companion computer | Orange Pi 5 (ARM64) |
| PX4 version | v1.14.3 |
| ROS 2 | Humble |

---

## Phase Checklist

### Phase 0 — Workspace Restructure ✅ COMPLETE
- [x] `Dockerfile.px4_sitl` — px4io base + PX4 v1.14.3 SITL baked in
- [x] `Dockerfile.as2`  — aerostack2/nightly-humble + apt AS2 packages
- [x] `Dockerfile.vio`  — ros:humble + apt OpenVINS
- [x] `Dockerfile.hw`   — ARM64 + librealsense v2.58.2 + AS2
- [x] `docker-compose.yml` — 4 services with profiles + health checks
- [x] `Makefile` — streamlined targets
- [x] `.env` — updated version pins
- [x] `quad_sim/config/` — world.yaml, AS2 platform/estimator/controller
- [x] `quad_sim/launch_as2.bash` — tmux-based node launcher
- [x] `quad_core/mission.py` — Aerostack2 Python API GPS-denied mission
- [x] `quad_core/config/ekf2_vio.params` — EKF2 GPS-denied parameters
- [x] `quad_core/config/vio_d435i.yaml` — OpenVINS D435i calibration
- [x] `quad_real/config/` — hardware platform / estimator / realsense config
- [x] `quad_real/launch_as2.bash` — hardware node launcher
- [x] `scripts/health_check.sh` — runtime verification
- [x] `scripts/upload_px4_params.py` — MAVLink parameter uploader

### Phase 1 — Build & Simulation Smoke Test
```bash
make build      # ~5-10 min first time (px4io base is heavy)
make sim-up     # starts px4_sitl + aerostack2
make health     # verify all green
make as2-launch # start AS2 nodes inside container
```
- [ ] All health checks pass
- [ ] `ros2 topic list` shows `/fmu/out/vehicle_odometry`
- [ ] `ros2 topic list` shows `/drone0/self_localization/pose`
- [ ] PX4 shows 'Ready to fly' in logs

### Phase 2 — Offboard Hover (No VIO)
- [ ] `make as2-launch` → all 5 AS2 windows healthy in tmux
- [ ] `make mission` → drone arms, takes off to 2m, hovers, lands
- [ ] No GPS — use ground_truth or barometer for initial test

### Phase 3 — GPS-Denied VIO Simulation
```bash
make vio-up     # adds OpenVINS to running stack
make params-upload  # upload ekf2_vio.params to PX4 SITL
```
- [ ] `/openvins/odometry` publishing at ~30Hz
- [ ] EKF2 fusing vision (check via QGroundControl)
- [ ] Complete 4-point square mission GPS-denied

### Phase 4 — Real Hardware (Orange Pi + Pixhawk + D435i)
- [ ] Build hw image on Orange Pi: `make build-hw`
- [ ] `make hw-up` → RealSense streams, serial DDS connects
- [ ] Upload `ekf2_vio.params` to physical Pixhawk
- [ ] Tethered hover test (props off first)
- [ ] `make mission` — identical `mission.py` runs on real drone
