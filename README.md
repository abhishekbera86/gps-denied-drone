# GPS-Denied Autonomous Drone

> PX4 v1.14.3 · ROS 2 Humble · Aerostack2 · OpenVINS VIO · Fully Docker-based

Flies a quadcopter **without GPS** using Visual-Inertial Odometry (RealSense D435i + OpenVINS).  
Identical autonomy code runs in simulation **and** on the real drone.

---

## Requirements

- Ubuntu 22.04 (or any Linux with Docker)
- Docker Engine + docker compose plugin
- No GPU required — simulation runs headless

```bash
# Install Docker (skip if already installed)
curl -fsSL https://get.docker.com | bash
sudo usermod -aG docker $USER
newgrp docker
```

---

## Quick Start

### Step 1 — Build (one time only, ~2-3 min)

```bash
git clone https://github.com/abhishekbera86/gps-denied-drone.git
cd gps-denied-drone
make build
```

### Step 2 — Launch the simulation world

```bash
make sim
```

This does everything automatically:
- Starts the Aerostack2 container (one container — no PX4, no Gazebo, no GPU)
- Builds the `quad_sim` and `quad_core` ROS 2 packages inside the container
- Launches all autonomy nodes via `ros2 launch quad_sim sim.launch.py`
- Prints **✓ All Aerostack2 nodes are running** when ready

This terminal **stays live** and shows all ROS 2 node logs.
Open a **second terminal** for the next step.

### Step 3 — Run the mission

```bash
make mission
```

The drone arms, takes off to 2 m, flies a 4-point GPS-denied square, and lands.

### Stop

```bash
make stop
```

---

## All Commands

```
make build      Build Docker images (first time only)

make sim        Launch simulation world (PX4 + Gazebo + AS2 nodes)
make mission    Run the autonomous mission
make stop       Stop everything

make vio        Launch sim with OpenVINS GPS-denied VIO stack
make hw         Start hardware stack (run on Orange Pi 5)

make shell      Open bash inside the Aerostack2 container
make logs       Tail logs from all containers
make health     Run health check
make ps         Show container status
```

---

## Project Structure

```
px4_docker_ws/
│
├── docker/
│   ├── Dockerfile.px4_sitl   PX4 SITL + Gazebo (pre-built, px4io base)
│   ├── Dockerfile.as2        Aerostack2 (pre-built, AS2 nightly-humble base)
│   ├── Dockerfile.vio        OpenVINS VIO
│   └── Dockerfile.hw         Hardware stack for Orange Pi 5 (ARM64)
│
├── ros2_ws/src/
│   ├── quad_core/            SHARED — same code in sim and real
│   │   ├── mission.py             ← the autonomous mission script
│   │   └── config/
│   │       ├── ekf2_vio.params    ← PX4 EKF2 GPS-denied parameters
│   │       └── vio_d435i.yaml     ← OpenVINS camera calibration
│   │
│   ├── quad_sim/             SIMULATION config only
│   │   └── config/
│   │       ├── world.yaml              ← Gazebo world + drone spawn
│   │       ├── as2_platform_sim.yaml   ← UDP bridge to PX4 SITL
│   │       ├── state_estimator_sim.yaml
│   │       └── motion_controller_sim.yaml
│   │
│   └── quad_real/            HARDWARE config only (Orange Pi 5)
│       └── config/
│           ├── as2_platform_hw.yaml    ← Serial bridge to Pixhawk
│           ├── state_estimator_hw.yaml
│           └── realsense_hw.yaml       ← RealSense D435i settings
│
└── scripts/
    ├── launch_sim.sh         Unified sim world launcher (called by make sim)
    ├── health_check.sh       Stack health verification
    └── upload_px4_params.py  Upload EKF2 params to PX4 via MAVLink
```

---

## Sim-to-Real

The **only** difference between simulation and real hardware is one line in the platform config:

| | Simulation | Real Hardware |
|---|---|---|
| Transport | `udp` (to PX4 SITL) | `serial` (to Pixhawk UART) |
| `mission.py` | ✅ Identical | ✅ Identical |
| VIO pipeline | ✅ Identical | ✅ Identical |
| EKF2 params | ✅ Identical | ✅ Identical |

---

## Hardware Setup (Orange Pi 5 + Pixhawk + RealSense D435i)

1. Connect RealSense D435i → USB 3.0
2. Connect Pixhawk TELEM2 → UART (`/dev/ttyUSB0`)
3. Flash PX4 params via QGroundControl: load `ros2_ws/src/quad_core/config/ekf2_vio.params`
4. On the Orange Pi:
   ```bash
   git clone https://github.com/abhishekbera86/gps-denied-drone.git
   cd gps-denied-drone
   make build    # builds ARM64 image (~15 min on Orange Pi)
   make hw       # launches hardware stack
   make mission  # identical mission.py
   ```

> **RealSense firmware**: 5.17.3.10 → librealsense SDK v2.58.2 (pre-installed in `Dockerfile.hw`)

---

## Troubleshooting

**`make sim` hangs waiting for PX4**
```bash
docker logs px4_sitl   # look for errors
```

**Topics not visible after `make sim`**
```bash
make health            # shows what's missing
make shell             # get into AS2 container
ros2 topic list        # check manually
```

**Mission fails to arm**
- PX4 needs ~10s after AS2 nodes start before it accepts OFFBOARD commands
- Run `make health` and wait for all green, then `make mission`

**Clean restart**
```bash
make stop && make sim
```
