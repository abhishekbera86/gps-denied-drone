# GPS-Denied Autonomous Drone Stack

> PX4 v1.17.0 · Gazebo Harmonic · ROS 2 Humble · Fully Containerized

A GPS-denied autonomous quadcopter stack built directly on **PX4's native ROS 2
integration** (the uXRCE-DDS bridge + `px4_msgs`/`px4_ros_com` — no MAVSDK, no
third-party mission framework). The goal is a quadcopter that flies without
GPS using Visual-Inertial Odometry from an Intel RealSense D435i, with the
exact same autonomy code running in Gazebo simulation and on a real Orange Pi
5 Plus + Pixhawk 6 companion-computer setup.

Everything runs inside Docker. **Nothing is installed on the host machine or
on the Orange Pi 5 Plus** — no ROS 2, no PX4 toolchain, no Gazebo, on either
machine. All of it lives in containers.

---

## 1. Objective

- **GPS-denied flight** using VIO from an Intel RealSense D435i
- **PX4 v1.17.0** flight stack, talking to ROS 2 over PX4's own uXRCE-DDS
  bridge — plain ROS 2 topics end to end, no MAVLink translation hop
- **ROS 2 Humble** throughout
- **Gazebo Harmonic** simulation, headless (no GPU required — this stack is
  developed on a GPU-less ThinkPad running Ubuntu 22.04)
- **Modular, pluggable missions** — adding a new flight pattern means adding
  one file, not editing a monolithic script
- **One autonomy codebase, two targets**: sim (Gazebo, dev host) and real
  hardware (Orange Pi 5 Plus + Pixhawk 6) run identical mission/control code
- A clean extension point for **SLAM and Nav2** once basic flight and VIO are
  solid (not yet built — see Roadmap below)

---

## 2. Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                     common_autonomy (ROS 2 workspace)                │
│           SHARED — identical source, sim and real hardware          │
│  common_control/  common_missions/  common_perception/               │
└───────────────────────────────┬──────────────────────────────────────┘
                                 │  identical ROS 2 topics
                                 │  (/fmu/in/*, /fmu/out/*, /vio/odometry…)
                 ┌───────────────┴────────────────┐
        ┌────────▼─────────┐            ┌─────────▼──────────┐
        │   sim_bringup     │            │    hw_bringup       │
        │  dev host, x86_64 │            │  Orange Pi 5+, ARM64│
        │  no GPU needed    │            │                     │
        │                   │            │  Not built yet —    │
        │  px4-sim:         │            │  see Roadmap        │
        │   PX4 v1.17.0 SITL│            │                     │
        │   + Gazebo Harmonic│           │                     │
        │   headless        │            │                     │
        │                   │            │                     │
        │  ros2-autonomy:   │            │                     │
        │   ROS 2 Humble +  │            │                     │
        │   uXRCE-DDS agent │            │                     │
        └───────────────────┘            └─────────────────────┘
```

**Rule:** all flight/mission logic lives in `common_autonomy` (under
`ros2_ws/src/`). Sim/hw bringup packages only contain launch files and
world/connection config. The only difference between sim and real is the
uXRCE-DDS transport endpoint (UDP vs serial) — never the mission code.

For the full phase-by-phase design (why each version pin was chosen, what
each future phase covers), see `IMPLEMENTATION_PLAN.md` in this repo — it's a
local working document (gitignored, not pushed) since it changes fast during
active development.

---

## 3. Prerequisites

### Host machine
- **OS**: Ubuntu 22.04 LTS or any Linux distribution running Docker
- **Architecture**: x86_64 for development/simulation; ARM64 (Orange Pi 5
  Plus) for real flight hardware
- **Graphics**: none required — Gazebo Harmonic runs fully headless
- **RAM**: 8 GB+ recommended (PX4 SITL compiles ~1140 objects on first build)

### Install Docker
```bash
curl -fsSL https://get.docker.com | bash
sudo usermod -aG docker $USER
newgrp docker
docker --version
docker compose version
```

No other host packages are required — not even `git` for PX4 or ROS 2; all
of that lives inside the build containers.

---

## 4. Quick Start

```bash
git clone https://github.com/abhishekbera86/gps-denied-drone.git
cd gps-denied-drone
make build      # first time only — compiles PX4 SITL + the ROS 2 bridge image
make sim        # starts px4-sim + ros2-autonomy
```

`make build` compiles two images:
- `px4-sim` — clones PX4 v1.17.0, installs Gazebo Harmonic via PX4's own
  `Tools/setup/ubuntu.sh`, and pre-compiles the `px4_sitl_default` SITL
  target (~2–3 minutes on a modern 4-core CPU once Gazebo/toolchain
  packages are cached; longer on the very first run).
- `ros2-autonomy` — ROS 2 Humble + the Micro-XRCE-DDS-Agent (built from
  source) + `px4_msgs`/`px4_ros_com` (colcon-built against PX4 v1.17.0
  message definitions).

`make sim` starts both containers. Check that PX4 booted cleanly:
```bash
make logs
```
You should see Gazebo Harmonic spawn the `x500_depth_0` model and PX4's
`uxrce_dds_client` connect to `127.0.0.1:8888`.

### Bridging PX4 to ROS 2

The Micro-XRCE-DDS-Agent isn't auto-started yet (Phase 1 will wire this into
a proper launch file). For now, start it manually and verify the bridge:

```bash
docker exec -d ros2-autonomy bash -c "MicroXRCEAgent udp4 -p 8888"

docker exec -it ros2-autonomy bash
source /opt/ros/humble/setup.bash
source /opt/px4_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list          # should show /fmu/in/* and /fmu/out/* topics
ros2 topic hz /fmu/out/vehicle_odometry   # should report ~10-15 Hz
```

### Stopping everything
```bash
make stop
```

---

## 5. Known Issues Hit During Bring-Up (already fixed in this repo)

Documenting these so a future rebuild-from-scratch doesn't waste time
rediscovering them — both are already fixed in the Dockerfiles in this repo.

**1. `DONT_RUN=1` does not stop the new Gazebo target from launching.**
PX4's `DONT_RUN` environment variable is only checked by the old Gazebo
Classic and jMAVSim `sitl_run.sh` scripts. The new Gazebo integration's
`gz_<model>` make targets (e.g. `gz_x500_depth`) always launch the
simulation as part of the build invocation — so
`DONT_RUN=1 make px4_sitl gz_x500_depth` inside a `docker build` step hangs
forever (Gazebo + PX4 just sit there running). Fixed in
`docker/Dockerfile.px4_sim` by pre-compiling with `make px4_sitl_default`
instead — the same compile-only target PX4's own CI uses
(`check_px4_sitl_default`, `coverity_scan`) — which builds everything
including the `gz_bridge` module without ever invoking a run step.

**2. `ros:humble-ros-base` doesn't ship `rmw_cyclonedds_cpp`.**
Setting `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` without installing the
matching package fails with `librmw_cyclonedds_cpp.so: cannot open shared
object file`. Fixed in `docker/Dockerfile.ros2_autonomy` by explicitly
installing `ros-humble-rmw-cyclonedds-cpp` (appended as its own `RUN` layer
*after* the expensive `colcon build` of `px4_msgs`/`px4_ros_com`, so editing
it doesn't invalidate that ~10-minute build cache).

**3. Don't use `px4io/px4-sitl-gazebo` or similar prebuilt PX4+Gazebo images
without checking their pinned PX4 version first.** One such image already
cached on this project's dev machine turned out to ship PX4 v1.18.0-alpha1
(an unreleased alpha) with no path back to a stable version — its apt repo
config had already been stripped from the image, leaving only the alpha
`.deb` installed. Always check `apt-cache policy px4-gazebo` (or equivalent)
before trusting a prebuilt image's version claim.

---

## 6. Repository Layout

```
docker/
  Dockerfile.px4_sim         PX4 v1.17.0 SITL + Gazebo Harmonic (headless)
  Dockerfile.ros2_autonomy   ROS 2 Humble + uXRCE-DDS agent + px4_msgs/px4_ros_com
docker-compose.yml           sim profile: px4-sim + ros2-autonomy (host networking)
.env                         Version pins (PX4, px4_msgs/px4_ros_com branches, Gazebo model)
Makefile                     build / sim / stop / shell / logs / ps
ros2_ws/src/                 common_control, common_missions, common_perception,
                              sim_bringup, hw_bringup — not yet populated (Phase 1+)
```

---

## 7. Command Reference

| Command | Action |
|---|---|
| `make build` | Build `px4-sim` + `ros2-autonomy` images |
| `make sim` | Start PX4 SITL (Gazebo Harmonic, headless) + the ROS 2 bridge container |
| `make shell` | Bash inside `ros2-autonomy` |
| `make shell-px4` | Bash inside `px4-sim` |
| `make logs` | Tail logs from both containers |
| `make ps` | Show container status |
| `make stop` | Stop and remove both containers |
| `make clean` | Remove locally built images |
| `make clean-all` | Remove images **and** the colcon build cache volume |

---

## 8. Roadmap

This repo is under active development. Phase 0 (PX4 SITL + Gazebo Harmonic +
ROS 2 bridge, everything above) is complete and verified. Still to come, in
order: an offboard hover control node, pluggable mission scripts, GPS-denied
VIO in simulation, then the same code running on the Orange Pi 5 Plus +
Pixhawk 6 + RealSense D435i, and finally SLAM/Nav2. Full phase-by-phase
detail lives in `IMPLEMENTATION_PLAN.md` (local, gitignored, not pushed —
it's an internal working document that changes too fast to keep in sync with
the public repo).
