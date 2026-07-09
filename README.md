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

## Table of Contents

1. [Objective](#1-objective)
2. [Architecture](#2-architecture)
3. [Prerequisites](#3-prerequisites)
4. [Step-by-Step Setup and Usage Guide](#4-step-by-step-setup-and-usage-guide)
   - [4.1 Clone the Repo](#41-clone-the-repo)
   - [4.2 Build the Docker Images](#42-build-the-docker-images)
   - [4.3 Start the Simulation](#43-start-the-simulation)
   - [4.4 Build the ROS 2 Workspace](#44-build-the-ros-2-workspace)
   - [4.5 Wait for PX4 to Be Ready to Arm](#45-wait-for-px4-to-be-ready-to-arm)
   - [4.6 Fly the Hover Test](#46-fly-the-hover-test)
   - [4.7 Fly a Mission](#47-fly-a-mission)
   - [4.8 Inspect the Running System](#48-inspect-the-running-system)
   - [4.9 Stop the Stack](#49-stop-the-stack)
5. [Command Reference](#5-command-reference)
6. [ROS 2 Topics Reference](#6-ros-2-topics-reference)
7. [ROS 2 Parameters Reference](#7-ros-2-parameters-reference)
8. [Localization Source: GPS or VIO](#8-localization-source-gps-or-vio)
9. [Launch Files Reference](#9-launch-files-reference)
10. [Environment Variables Reference](#10-environment-variables-reference)
11. [No Upstream Repos Are Forked or Modified](#11-no-upstream-repos-are-forked-or-modified)
12. [Known Issues Hit During Bring-Up](#12-known-issues-hit-during-bring-up)
13. [Repository Layout](#13-repository-layout)
14. [Roadmap](#14-roadmap)

---

## 1. Objective

- **GPS-denied flight** using VIO from an Intel RealSense D435i
- **PX4 v1.17.0** flight stack, talking to ROS 2 over PX4's own uXRCE-DDS
  bridge — plain ROS 2 topics end to end, no MAVLink translation hop
- **ROS 2 Humble** throughout
- **Gazebo Harmonic** simulation — headless by default (no GPU required), with
  an opt-in real GUI window to watch flights visually (§4.3)
- **Modular, pluggable missions** — adding a new flight pattern means adding
  one file, not editing a monolithic script
- **One autonomy codebase, two targets**: sim (Gazebo, dev host) and real
  hardware (Orange Pi 5 Plus + Pixhawk 6) run identical mission/control code
- A clean extension point for **SLAM and Nav2** once basic flight and VIO are
  solid (not yet built — see [Roadmap](#14-roadmap))

---

## 2. Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                     common_autonomy (ROS 2 workspace)                │
│           SHARED — identical source, sim and real hardware          │
│  common_control/  common_missions/  common_perception/ (VIO, §8)     │
└───────────────────────────────┬──────────────────────────────────────┘
                                 │  identical ROS 2 topics
                                 │  (/fmu/in/*, /fmu/out/*, /vio/odometry…)
                 ┌───────────────┴────────────────┐
        ┌────────▼─────────┐            ┌─────────▼──────────┐
        │   sim_bringup     │            │    hw_bringup       │
        │  dev host, x86_64 │            │  Orange Pi 5+, ARM64│
        │  GPU optional     │            │                     │
        │  (GUI opt-in)     │            │  STUB — untested,   │
        │  px4-sim:         │            │  no hardware yet.   │
        │   PX4 v1.17.0 SITL│            │  Serial uXRCE-DDS   │
        │   + Gazebo Harmonic│           │  agent + realsense/ │
        │   headless or GUI │            │  perception slots — │
        │                   │            │  see §14 Roadmap    │
        │  ros2-autonomy:   │            │                     │
        │   ROS 2 Humble +  │            │                     │
        │   uXRCE-DDS agent │            │                     │
        └───────────────────┘            └─────────────────────┘
```

**Rule:** all flight/mission logic lives in `common_autonomy` (under
`ros2_ws/src/`: `common_control`, `common_missions`). Sim/hw bringup packages
(`sim_bringup`, `hw_bringup`) only contain launch files and world/connection
config — never flight logic. The only difference between sim and real is the
uXRCE-DDS transport endpoint (UDP vs serial) and, from Phase 3 on, the sensor
source topic — never the mission code.

**How a flight actually happens, end to end:**
1. A mission node (e.g. `square_mission`) publishes an `OffboardControlMode`
   heartbeat + `TrajectorySetpoint` at 10 Hz, and sends `VehicleCommand`s to
   arm and switch modes — all on `/fmu/in/*` topics.
2. The Micro-XRCE-DDS-Agent (running inside `ros2-autonomy`) forwards those
   ROS 2 messages over UDP to PX4's `uxrce_dds_client`, which is compiled
   into the flight stack itself (`px4-sim` in sim; the real Pixhawk 6 in
   Phase 4).
3. PX4 flies the vehicle and publishes its own state back out on `/fmu/out/*`
   topics (`vehicle_status_v1`, `vehicle_local_position_v1`, …), which flow
   back through the same DDS bridge to the mission node.
4. The mission node's state machine reads that feedback to decide when it's
   armed, when it's reached a waypoint, and when to land — see
   [§6 ROS 2 Topics Reference](#6-ros-2-topics-reference) for the exact list.

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
- **Graphics**: none required for headless mode (`make sim`) — Gazebo
  Harmonic runs fully headless by default. For GUI mode (`make sim-gui`, §4.3)
  you need an X11 desktop session; any halfway-recent Intel iGPU via Mesa is
  enough (verified on a ThinkPad W541's HD 4600) — no dedicated/discrete GPU
  or proprietary driver required.
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

## 4. Step-by-Step Setup and Usage Guide

This walks through the entire flow on any Linux box that has Docker, from a
fresh clone to a flown mission, one command at a time. Every command's exact
effect is explained inline; full reference tables (all commands, all ROS
topics, all parameters, all launch files) follow in §5–§9.

### 4.1 Clone the Repo

```bash
git clone https://github.com/abhishekbera86/gps-denied-drone.git
cd gps-denied-drone
```

This is the *only* repo you ever clone or touch. PX4-Autopilot, px4_msgs,
px4_ros_com, and the Micro-XRCE-DDS-Agent are all cloned automatically,
read-only, at pinned versions, inside the Docker build — see
[§11](#11-no-upstream-repos-are-forked-or-modified).

### 4.2 Build the Docker Images

```bash
make build
```

Compiles two images (expect ~15–40 min on the very first run, depending on
CPU and network — later rebuilds hit Docker layer cache and take seconds):

- **`px4-sim`** — clones PX4-Autopilot at the pinned `v1.17.0` tag, installs
  Gazebo Harmonic via PX4's own `Tools/setup/ubuntu.sh`, pre-compiles the
  `px4_sitl_default` target, and overlays this repo's SITL-only param file
  (§12 issue 4) and empty world (§12 issue 13).
- **`ros2-autonomy`** — ROS 2 Humble + the Micro-XRCE-DDS-Agent (built from
  source at `v3.0.1`) + `px4_msgs` (`release/1.17`) / `px4_ros_com`
  (`release/1.16`), colcon-built against PX4 v1.17.0 message definitions.

Rebuild an individual image (e.g. after editing a Dockerfile) with
`docker compose --profile sim build <service>`, e.g.
`DOCKER_BUILDKIT=1 docker compose --profile sim build px4-sim`.

### 4.3 Start the Simulation

You have two choices here — same underlying stack, different Gazebo output.

#### Headless mode (default — no GPU or display needed)

```bash
make sim
```

Starts both containers with Gazebo running server-only (no window). This is
the fastest path and works over SSH, in CI, or on a machine with no GPU at
all. The `ros2-autonomy` entrypoint auto-starts the Micro-XRCE-DDS-Agent, so
the PX4 ↔ ROS 2 bridge comes up on its own — no manual step.

#### GUI mode (watch the drone fly)

```bash
make sim-gui
```

Starts the *exact same* stack, but pops the real Gazebo Harmonic window on
your desktop so you can watch takeoff and missions live. Mechanically, this
layers `docker-compose.gui.yml` on top of the base compose file: it forwards
your host's X11 socket (`/tmp/.X11-unix`) and `/dev/dri` (GPU render nodes)
into the `px4-sim` container, and flips PX4's Gazebo launch from server-only
to server+GUI. `make sim-gui` also runs `xhost +local:root` for you so the
container is allowed to draw on your X display.

`make sim-gui` also starts a third container, `rqt-viewer` — a live
`rqt_image_view` window subscribed to `/camera/camera/color/image_raw`
from the moment the stack comes up, not launched manually per mission.
This matters beyond convenience: it subscribes *before* any mission's
camera bridge even exists, so DDS discovery connects the two the instant a
`VIO_BACKEND=openvins` mission actually starts publishing, regardless of
how long you wait to launch one. Starting a viewer only *after* a mission
(the old manual `docker run` workflow) can lose that race entirely — a
mission's own process-exit path (§12 issue 25) tears its camera bridge
down the instant the mission finishes, so a late viewer might never see a
single frame. The window stays blank/black until a vision mission is
actually flying; that's expected, not a bug — the GPS localization path
never starts a camera bridge at all. `make stop` tears it down along with
everything else.

**Choosing a world** — two exist, pick with `PX4_GZ_WORLD=`:

```bash
PX4_GZ_WORLD=empty make sim-gui       # default — bare ground+sun+grid, open space
PX4_GZ_WORLD=vio_test make sim-gui    # required for VIO_BACKEND=openvins (§8)
```

`empty` (`docker/px4_sitl_worlds/empty.sdf`) is open space with nothing to
watch takeoff/missions through. `vio_test`
(`docker/px4_sitl_worlds/vio_test.sdf`) adds a ring of colored boxes plus a
ground-level checkerboard — **required, not cosmetic**, if you're flying
`VIO_BACKEND=openvins`: monocular VIO needs real corner features to track,
and the plain `empty` world's flat gray plane gives it zero (OpenVINS's
initializer fails every frame). Both are repo-owned overlay files, not a
fork of PX4 (§11) — same mechanism as the SITL param override, §12 issue 4.

**Setting `PX4_GZ_WORLD` in `.env` instead of on the command line does
not work and will not error** — the Makefile's own default is meant to be
overridden per-invocation (`PX4_GZ_WORLD=vio_test make sim-gui`), and a
value hardcoded in `.env` silently wins over that override every time (a
GNU Make quirk — confirmed as a real, hit bug, see §12 issue 19). Always
pass it on the command line, never edit it into `.env`.

If the window doesn't appear, or Gazebo crashes trying to use your GPU, force
software rendering instead:
```bash
GZ_SW_RENDER=1 make sim-gui
```

**If the Gazebo window opens and the world (props/checkerboard) renders,
but the drone itself never appears** — check the entity/scene tree panel
(usually the left sidebar) for `x500_depth_0`. If it's missing there too
even though `make logs`/`ros2 topic list` show the vehicle is actually
flying, this is a GUI-client scene-sync issue, not a simulation problem —
the vehicle is spawned dynamically by PX4 *after* the world/GUI are already
up, and on a heavily loaded host the GUI client can miss that event and
never recover on its own. Fix it without touching the running sim:
```bash
make gz-resync
```
This restarts **only the GUI client** (a pure viewer) — the sim server,
PX4, and any in-progress flight are unaffected. The fresh client downloads
the complete scene on connect, drone included. See §12 issue 20 for the
full diagnosis, including why this happens more on slow hosts.

`make sim` (headless) is completely unaffected by any of this — same image,
just a different compose overlay, and confirmed to have zero GUI processes
and zero X11 errors regardless of which mode you last used.

Either way, verify the stack came up with:
```bash
make logs
```
You should see Gazebo Harmonic spawn `x500_depth_0` and `uxrce_dds_client`
connect to `127.0.0.1:8888`.

### 4.4 Build the ROS 2 Workspace

```bash
make build-ws
```

Colcon-builds `common_control`, `common_missions`, `common_perception`,
`sim_bringup`, and `hw_bringup` inside `ros2-autonomy` with
`--symlink-install`, into the persistent `/ros2_ws_build` volume. Source is
live-mounted from
`ros2_ws/src/` on the host, and `--symlink-install` means the install tree
is symlinked back to that source rather than copied — so **you only need to
re-run this after adding/removing a package or editing a `setup.py`/
`package.xml`**. Editing a mission's Python, a launch file, or a params YAML
(§7) takes effect on the *next* `make flight-test`/`make mission` with no
rebuild at all — `make mission` and `make flight-test` deliberately don't
colcon-build for you every time, so tuning-and-reflying stays fast.

If you skip this step, `make flight-test`/`make mission` will tell you to
run it — they check for the built workspace rather than silently failing.

### 4.5 Wait for PX4 to Be Ready to Arm

PX4's EKF2 state estimator needs **~30–60 seconds** after boot to converge
before it will allow arming — this is normal PX4 behavior, not something
wrong with this repo (§12 issue 8). You don't have to do anything: every
flight command below retries once per second until PX4 accepts. If you want
to watch convergence yourself:
```bash
make shell
ros2 topic echo /fmu/out/vehicle_status_v1 --once
# wait for: pre_flight_checks_pass: true
```

### 4.6 Fly the Hover Test

In a second terminal (leave the sim running in the first):
```bash
make flight-test
```

Runs `ros2 launch sim_bringup sim.launch.py action:=hover`, which:

1. Streams the `OffboardControlMode` heartbeat + `TrajectorySetpoint` at 10 Hz
2. Switches PX4 to offboard mode and arms (retrying once per second until
   PX4 confirms via `vehicle_status` — see §4.5)
3. Takes off to 2 m, hovers 5 s, lands, and disarms

Expected output ends with:
```
[INFO] [...]: Armed and offboard — climbing to takeoff height
[INFO] [...]: Reached takeoff height — hovering for 5.0s
[INFO] [...]: Hover complete — landing
[INFO] [...]: Landed and disarmed — mission complete
```
(For `LOCALIZATION=vision VIO_BACKEND=openvins`, you may instead see a
`WARN` line about the "low-throttle fallback" engaging just before that —
that's expected and by design, not an error; see §8.)

By default this flies GPS. To fly it GPS-denied instead (needs
`PX4_GZ_WORLD=vio_test`, §4.3):
```bash
LOCALIZATION=vision VIO_BACKEND=openvins make flight-test
```
See [§8](#8-localization-source-gps-or-vio) for what these two variables do.

### 4.7 Fly a Mission

```bash
make mission                    # square, the default and currently only mission, GPS
LOCALIZATION=vision VIO_BACKEND=openvins make mission MISSION=square    # GPS-denied
```

Runs `ros2 launch sim_bringup sim.launch.py action:=mission
mission:=<name>`. `square` (exact geometry — `sim_params.yaml`, §7) is the
only mission in the repo right now — a square flight path at takeoff
height, nose pointed along each leg, landing back at the start. (A
`survey` lawnmower-coverage mission existed earlier but was removed
2026-07-09 — its indoor test geometry didn't suit its own footprint;
a differently-shaped mission against a purpose-built world is planned
separately, not a retraction of the pattern itself.)

`square` is a subclass of `MissionBase`, which reuses the entire hover-test
arm/offboard/takeoff/land state machine and only supplies waypoints — a new
mission is ~25 lines (declare geometry parameters, return a waypoint list;
see `ros2_ws/src/common_missions/`). Waypoint arrival is judged by position
tolerance (0.5 m default), never by elapsed time, so missions behave
identically on slow and fast hosts. Mission geometry and control tuning live
in `sim_bringup/config/sim_params.yaml` (§7) — **every one of these values is
required, with no hardcoded fallback in the Python**: edit the YAML and
re-run `make mission`, no rebuild needed (§4.4). If a value is missing from
the YAML, the node fails loudly at startup with a `[FATAL]` log naming the
exact missing parameter, instead of silently flying an unintended default.

### 4.8 Inspect the Running System

```bash
make shell             # bash inside ros2-autonomy (ROS 2 side)
```
From inside that shell:
```bash
ros2 topic list                                    # every topic on the bridge
ros2 topic echo /fmu/out/vehicle_status_v1 --once   # one-shot state snapshot
ros2 topic hz /fmu/out/vehicle_odometry             # confirm ~10-15 Hz data flow
ros2 node list                                      # currently running nodes
```
See [§6](#6-ros-2-topics-reference) for the full topic list this stack
actually uses.

```bash
make shell-px4          # bash inside px4-sim (flight-controller side)
```
From inside that shell, PX4 internals are queryable via the `px4-*` client
binaries:
```bash
cd /PX4-Autopilot/build/px4_sitl_default/rootfs
/PX4-Autopilot/build/px4_sitl_default/bin/px4-listener vehicle_status
/PX4-Autopilot/build/px4_sitl_default/bin/px4-listener health_report
```

### 4.9 Stop the Stack

```bash
make stop
```

**Always restart the whole stack together** (`make stop && make sim`, or
`make stop && make sim-gui`). Restarting only the `px4-sim` container leaves
the DDS bridge wedged — see §12 issue 5.

---

## 5. Command Reference

Every `make` target, what it does, and any parameters it accepts. Parameters
are passed as `VAR=value` after the target name (standard `make` override
syntax) or as an environment variable before it.

| Command | Parameters | What it does |
|---|---|---|
| `make build` | — | Builds the `px4-sim` and `ros2-autonomy` **Docker images**. One-time, or after editing a Dockerfile. |
| `make build-ws` | — | Colcon-builds the **ROS 2 workspace** inside `ros2-autonomy`. One-time, or after adding/removing a package or editing `setup.py`/`package.xml` — NOT needed after editing Python/launch/config files (§4.4). |
| `make sim` | `PX4_GZ_WORLD=empty` *(default)* or `vio_test` | Starts PX4 SITL (Gazebo Harmonic, **headless**) + the ROS 2 bridge container. |
| `make sim-gui` | `PX4_GZ_WORLD=` (same as above) · `GZ_SW_RENDER=1` *(optional)* — forces software (llvmpipe) rendering if the GPU path fails | Same stack as `make sim`, but with the real Gazebo GUI window forwarded to the host X display, plus a third container (`rqt-viewer`) showing a live `/camera/camera/color/image_raw` preview, up from the start so it can't lose the sequencing race against a mission's camera bridge (§4.3). **Always pass `PX4_GZ_WORLD` on the command line, never edit it into `.env`** — see §12 issue 19. |
| `make flight-test` | `LOCALIZATION=gps` *(default)* or `vision` · `VIO_BACKEND=loopback` *(default)* or `openvins` (only when `LOCALIZATION=vision`, and only with `PX4_GZ_WORLD=vio_test` — §8) | Flies the hover test (arm → takeoff → hover → land) via `sim_bringup`. Requires `make build-ws` first. |
| `make mission` | `MISSION=square` *(default, and currently the only mission)* · `LOCALIZATION=` / `VIO_BACKEND=` (same as above) | Flies the named waypoint mission via `sim_bringup`. Requires `make build-ws` first. |
| `make gz-resync` | — | Fallback: forces the Gazebo GUI to rebroadcast its full scene, for the rare case the drone doesn't appear in the GUI's 3D view (§4.3, §12 issue 20). No effect on headless mode. |
| `make shell` | — | Opens a bash shell inside `ros2-autonomy` (the ROS 2 / DDS-bridge container). |
| `make shell-px4` | — | Opens a bash shell inside `px4-sim` (the PX4 flight-controller container). |
| `make logs` | — | Tails logs from both containers (`Ctrl-C` to stop tailing; containers keep running). |
| `make ps` | — | Shows container status (`docker compose ps`). |
| `make stop` | — | Stops and removes both containers. Data in the colcon build cache volume survives. |
| `make clean` | — | Removes the locally built `px4-sim`/`ros2-autonomy` images (keeps the colcon cache volume). |
| `make clean-all` | — | Removes images **and** the colcon build cache volume — the next build/run starts completely fresh. |

`.env` also drives several build/runtime values (Gazebo model, world, version
pins) without needing a `make` parameter at all — see
[§9](#9-environment-variables-reference).

---

## 6. ROS 2 Topics Reference

Every ROS 2 topic that `common_control`/`common_missions` actually publishes
or subscribes to, over the PX4 uXRCE-DDS bridge.

| Topic | Direction | Message Type | Rate | Purpose |
|---|---|---|---|---|
| `/fmu/in/offboard_control_mode` | Publish | `px4_msgs/msg/OffboardControlMode` | 10 Hz (`control_rate_hz`) | Heartbeat telling PX4 "position control, offboard, still here" — required continuously or PX4 drops offboard mode. |
| `/fmu/in/trajectory_setpoint` | Publish | `px4_msgs/msg/TrajectorySetpoint` | 10 Hz | The current target `(x, y, z, yaw)` in NED, streamed even while disarmed (§12 issue 7). |
| `/fmu/in/vehicle_command` | Publish | `px4_msgs/msg/VehicleCommand` | On demand | Arm/disarm, offboard mode switch, and `NAV_LAND` commands. |
| `/fmu/out/vehicle_local_position_v1` | Subscribe | `px4_msgs/msg/VehicleLocalPosition` | ~as published by PX4 | Feeds the waypoint-arrival check (`waypoint_tolerance_m` distance test) and the takeoff-height check. |
| `/fmu/out/vehicle_status_v1` | Subscribe | `px4_msgs/msg/VehicleStatus` | ~as published by PX4 | `arming_state` and `nav_state` — drives every state-machine transition (armed? in offboard? disarmed after landing?). |
| `/fmu/out/vehicle_land_detected` | Subscribe | `px4_msgs/msg/VehicleLandDetected` | ~as published by PX4 | Only `has_low_throttle` is read (a pure actuator-thrust signal) — drives the post-landing disarm fallback (§7, §8) when PX4's own `arming_state` doesn't flip on its own. |

Notes:
- The `_v1` suffix is not a typo — PX4 v1.17 publishes **versioned** topic
  names for any message carrying a `MESSAGE_VERSION` field. The unversioned
  names used in older PX4 examples receive nothing on this version (§12
  issue 6).
- QoS on every topic above is `BEST_EFFORT` reliability + `TRANSIENT_LOCAL`
  durability + `KEEP_LAST` depth 1 (matching PX4's own DDS QoS) — see
  `PX4_QOS` in `common_control/common_control/offboard_control_node.py` if
  you're writing a new node against this bridge.

### `common_perception` topics (§8 — localization source)

Not used by `common_control`/`common_missions` — read/written by the
localization-source mechanism only, which is why they're broken out
separately rather than mixed into the table above.

| Topic | Direction | Message Type | Purpose |
|---|---|---|---|
| `/fmu/in/vehicle_visual_odometry` | Publish | `px4_msgs/msg/VehicleOdometry` | What EKF2 actually fuses as "vision" — published by `loopback_odometry_bridge` (`VIO_BACKEND=loopback`) or `openvins_odometry_bridge` (`VIO_BACKEND=openvins`), same target topic either way. |
| `/fmu/out/vehicle_odometry` | Subscribe | `px4_msgs/msg/VehicleOdometry` | PX4's own current fused estimate — `loopback_odometry_bridge`'s input (the Milestone A fake-VIO stand-in; zero-drift, no camera needed). |
| `/fmu/out/estimator_status_flags` | Subscribe (diagnostic) | `px4_msgs/msg/EstimatorStatusFlags` | Per-source fusion booleans (`cs_gnss_pos`, `cs_ev_pos`, …) — the ground truth for which source EKF2 is *actually* using, independent of what was requested. |

**`VIO_BACKEND=openvins` only** — the real Milestone B pipeline, bridged
from the simulated D435i camera through OpenVINS and back into PX4:

| Topic | Direction | Message Type | Purpose |
|---|---|---|---|
| `/camera/camera/color/image_raw` | `ros_gz_bridge` → OpenVINS | `sensor_msgs/msg/Image` | RGB feed, bridged from Gazebo's simulated D435i — same topic name real `realsense-ros` produces. |
| `/camera/camera/imu` | `ros_gz_bridge` → OpenVINS | `sensor_msgs/msg/Imu` | The D435i's OWN onboard IMU (co-located with the color sensor in the simulated `d435i` model) — matches what real hardware's `realsense-ros` publishes on this exact topic name. |
| `/camera/camera/color/camera_info` | `ros_gz_bridge` → OpenVINS | `sensor_msgs/msg/CameraInfo` | Intrinsics for the color stream. |
| `/camera/camera/depth/image_rect_raw` | `ros_gz_bridge` (bridged, not consumed) | `sensor_msgs/msg/Image` | Bridged for topic-name parity/future Nav2 use — OpenVINS's mono VIO doesn't consume it. Encoding differs from real hardware (sim: float meters; D435i: `16UC1` mm) — not reconciled. |
| `/ov_msckf/odomimu` | OpenVINS → `openvins_odometry_bridge` | `nav_msgs/msg/Odometry` | OpenVINS's own VIO estimate (ENU/FLU, arbitrary non-North-aligned yaw) — converted to NED/FRD and republished as `/fmu/in/vehicle_visual_odometry` above. |

---

## 7. ROS 2 Parameters Reference

Every `declare_parameter()` in the autonomy code, and what it controls.
**None of these have a hardcoded fallback in Python** — every value is
declared with `Parameter.Type.DOUBLE` (a type, not a value) via the shared
`_require_param()` helper in `OffboardControlNode`
(`common_control/common_control/offboard_control_node.py`), so it MUST come
from a launch `params_file` or an explicit `-p` override. If one is missing,
node startup fails immediately with a `[FATAL]` log naming the exact
parameter and the section it should have been in — never a silent flight
with an unintended value. In sim, values come from
`sim_bringup/config/sim_params.yaml`; the (untested, Phase 4) hardware
equivalent is `hw_bringup/config/hw_params.yaml` with more conservative
values. Edit the YAML to retune — no code change and no rebuild needed
(§4.4).

### How the mission parameter file is structured

Both YAML files use one **top-level section per mission**, named exactly
after that mission's ROS 2 node name (the string each mission passes to
`super().__init__(...)`, e.g. `square_mission`). This is standard ROS 2
parameter-file syntax — at launch, each node automatically reads only the
section whose name matches itself, so every mission's tuning can live in one
file without conflicting:

```yaml
# sim_bringup/config/sim_params.yaml
offboard_control_node:        # the hover profile (action:=hover)
  ros__parameters:
    takeoff_height_m: 2.0
    hover_seconds: 5.0
    control_rate_hz: 10.0
    waypoint_tolerance_m: 0.5
    max_velocity_m_s: 1.0                   # ← lower this if the vehicle moves too fast
    land_disarm_low_throttle_dwell_s: 3.0   # post-landing disarm fallback — see below
    land_disarm_max_timeout_s: 60.0

square_mission:                # action:=mission mission:=square
  ros__parameters:
    takeoff_height_m: 2.0
    waypoint_tolerance_m: 0.5
    max_velocity_m_s: 0.8      # NOT 0.2 — see the note below this table
    land_disarm_low_throttle_dwell_s: 3.0
    land_disarm_max_timeout_s: 60.0
    side_length_m: 3.0         # ← change this, then `make mission MISSION=square`
```

`square` is currently the only mission section — this per-mission-section
structure is what makes adding another one (or bringing back something
like the earlier `survey` lawnmower pattern, against its own purpose-built
world) a config/subclass addition, not a restructuring.

**To change a mission's behavior:** edit the numbers under its section and
re-run `make mission MISSION=<name>` — this is a launch-time file, not
compiled in, so no code change and no rebuild is needed. `hw_bringup/config/
hw_params.yaml` mirrors this exact structure with real-flight values.

**Future extension point (not implemented yet):** this one-file,
one-section-per-mission shape is deliberately simple so that a planned
operator app can later generate/edit this same YAML — pick a mission, adjust
its parameters in a UI, then launch with
`ros2 launch sim_bringup sim.launch.py params_file:=<generated path>` —
without needing to redesign the config format itself.

### `common_control` (base class for every mission — `offboard_control_node`)

| Parameter | Type | `sim_params.yaml` | `hw_params.yaml` | Meaning |
|---|---|---|---|---|
| `takeoff_height_m` | float | `2.0` | `1.5` | NED climb target, meters above ground. |
| `hover_seconds` | float | `5.0` | `5.0` | Hover dwell time for `action:=hover` (the plain flight-test). Ignored once a mission with waypoints is flying. |
| `control_rate_hz` | float | `10.0` | `10.0` | Publish rate for the heartbeat + setpoint control loop. |
| `waypoint_tolerance_m` | float | `0.5` | `0.7` | Arrival radius for both the takeoff-height check and mission waypoints. |
| `max_velocity_m_s` | float | `1.0` (hover) / `0.8` (missions) | `0.5` (hover) / `0.1` (missions) | Cruise speed cap toward any setpoint (takeoff, hover position, or a waypoint), in m/s. Without this, PX4's own offboard position controller accelerates toward every setpoint at its internal velocity limit — much faster than useful for watching a mission or a first real flight. Implemented as a feed-forward velocity vector aimed at the target, magnitude-capped at this value (`_capped_velocity_toward` in `offboard_control_node.py`) — not a PX4 firmware parameter. **Don't set this below ~0.5 m/s for `VIO_BACKEND=openvins` flights** — a real, confirmed failure mode: too slow starves monocular VIO of the acceleration events it needs to keep its scale/bias estimate converged, and a long, near-constant-velocity flight lets that error build up for the whole mission (§8, §12 issue 17). |
| `land_disarm_low_throttle_dwell_s` | float | `3.0` | `4.0` | Post-landing disarm fallback (§8, §12 issue 21): how long PX4's own actuator thrust must stay continuously low before this node explicitly force-disarms, used only if PX4's own auto-disarm doesn't happen on its own. Any single higher-throttle reading resets this to zero. |
| `land_disarm_max_timeout_s` | float | `60.0` | `90.0` | Ceiling on how long to wait for the condition above — if exceeded, this node gives up rather than ever disarming based on elapsed time alone, and logs an error requiring manual `px4-commander disarm -f`. Never the trigger by itself, only a bound on the wait. |
| `geofence_margin_m` | float | `2.0` | `1.0` | Geofence (§12 issue 24): horizontal margin added on every side of the current route's bounding box (origin + every queued waypoint — auto-derived, not a separately hand-maintained box; see `_geofence_bounds` in `offboard_control_node.py`). A position outside this box in any flying state (TAKEOFF/WAYPOINTS/HOVER) aborts immediately to `LAND`. |
| `geofence_height_margin_m` | float | `1.5` | `1.0` | Geofence altitude cap: how far above the highest point the route actually visits (usually `takeoff_height_m`) the vehicle may climb before the same abort triggers. |

### `square_mission`

| Parameter | Type | `sim_params.yaml` | `hw_params.yaml` | Meaning |
|---|---|---|---|---|
| `side_length_m` | float | `3.0` | `2.0` | Side length of the square flight path, in meters. |

Override any of these ad hoc without touching the YAML, e.g. for a quick
one-off test (this satisfies the required-parameter check just like the YAML
does):
```bash
ros2 run common_missions square_mission --ros-args -p side_length_m:=6.0
```

### What a missing parameter looks like

Delete or misspell a value the launched node needs and startup fails loudly
instead of silently flying a default:
```
[FATAL] [square_mission]: Required parameter 'side_length_m' was not
provided to node 'square_mission'. Launch with a params_file that has a
'square_mission:' section setting 'side_length_m' — see
sim_bringup/config/sim_params.yaml — or pass '-p side_length_m:=<value>'
directly.
```
followed by a Python traceback and a non-zero exit code — `ros2 launch`
reports the node as crashed rather than continuing.

---

## 8. Localization Source: GPS or VIO

Choose the drone's position/velocity source **before a mission starts** —
outdoors → GPS; indoors, GPS-denied → the Intel RealSense D435i camera drives
Visual-Inertial Odometry (VIO) instead. This is a launch-time choice, not a
live in-flight failover (that's future work). Full design rationale —
including the dead ends ruled out — lives in
`resource/phase3-gps-denied-localization-source.md`; this section is the
day-to-day usage reference.

```bash
make mission MISSION=square LOCALIZATION=gps                                       # default
make mission MISSION=square LOCALIZATION=vision                                    # Milestone A fake-VIO stand-in
make mission MISSION=square LOCALIZATION=vision VIO_BACKEND=openvins               # Milestone B real VIO
```

**`VIO_BACKEND=openvins` needs `PX4_GZ_WORLD=vio_test`** (§4.3) — start the
sim with `PX4_GZ_WORLD=vio_test make sim` (or `sim-gui`) *before* running
the command above. The default `empty` world has nothing for monocular VIO
to track and OpenVINS's initializer will fail every frame.

### Why this needs a MAVLink side-channel

Every other command in this repo talks to PX4 over its native ROS 2 /
uXRCE-DDS bridge — but that bridge **cannot set PX4 parameters at all** in
this pinned PX4 version (confirmed by exhaustive search of PX4's
`dds_topics.yaml`: zero `Parameter*` topics are bridged). Choosing the
localization source means setting PX4's `EKF2_GPS_CTRL`/`EKF2_EV_CTRL`
parameters, so a small one-shot script, `set_localization_source`
(`common_perception`), does this over a separate **MAVLink `PARAM_SET`
side-channel** — PX4 SITL already exposes a MAVLink UDP port independently
of the DDS agent, so this needs no PX4 rebuild. It runs once, before the
mission node starts, sequenced by the launch file
(`RegisterEventHandler(OnProcessExit(...))`) — not a persistent service.

### What actually gets set

| `LOCALIZATION=` | `EKF2_GPS_CTRL` | `EKF2_EV_CTRL` | `EKF2_EV_POS_X/Y/Z` | Height reference |
|---|---|---|---|---|
| `gps` (default) | `7` (HPOS+VPOS+VEL) | `0` (off) | untouched | GPS |
| `vision` | `0` (off) | `5` (HPOS+VEL) | `0.12 / -0.03 / -0.242` | Baro (automatic fallback) |

`EKF2_HGT_REF` (which height source PX4 treats as primary) is **never
touched** — changing it requires a PX4 reboot, and disabling GPS already
makes PX4's own automatic height-source fallback pick the next enabled
source (baro, on by default) with no reboot needed. Vision only supplies
horizontal position + velocity; yaw is left to the magnetometer/gyro to
avoid vision-yaw drift. This is standard practice for indoor PX4 setups.

`EKF2_EV_POS_X/Y/Z` (FRD, meters, `vision` only) tells EKF2 where the
vision sensor physically sits relative to the flight controller's own IMU —
the published vision odometry represents the simulated D435i's *own*
onboard IMU (co-located with its color sensor), not `base_link`, and
without this offset EKF2 silently assumes they're the same point. Getting
this wrong specifically corrupts attitude-change maneuvers (landing
corrections have more of those than steady cruise) — a real, confirmed
contributor to a landing-divergence bug this project hit (§12 issue 18).

### Confirming the switch actually took

`/fmu/out/estimator_status_flags` (already DDS-bridged, no gap to fill)
carries per-source fusion booleans — the ground truth for what PX4 is
*actually* fusing, independent of what was asked for:

```bash
ros2 topic echo /fmu/out/estimator_status_flags --once
# LOCALIZATION=vision should show:
#   cs_gnss_pos: false   cs_gnss_vel: false   (GPS genuinely off)
#   cs_ev_pos: true      cs_ev_vel: true      (vision genuinely fusing)
```

### The VIO pipeline itself — `VIO_BACKEND=`

`LOCALIZATION=vision` also starts whichever node publishes
`/fmu/in/vehicle_visual_odometry` (`px4_msgs/VehicleOdometry`) — the topic
PX4's EKF2 actually fuses vision from. `VIO_BACKEND=` picks which one:

- **`loopback`** (default) — `loopback_odometry_bridge` (`common_perception`)
  republishes PX4's own current estimate (`/fmu/out/vehicle_odometry`) back
  in as a zero-drift, always-available stand-in. This proves the entire
  switch → EKF2 → mission-flies-unmodified mechanism without needing a
  camera or VIO estimator at all — verified by flying `square` fully
  vision-fused and landing normally. No extra build step.
- **`openvins`** — the real Milestone B pipeline: a `ros_gz_bridge` remaps
  the simulated D435i's own camera + its own onboard IMU (a locally-authored
  Gazebo model, `docker/px4_sitl_models/d435i` — §11 — not PX4's stock
  camera) to the exact ROS 2 topic names/types the real RealSense D435i's
  `realsense-ros` driver produces (`/camera/camera/color/image_raw`,
  `/camera/camera/imu` — so sim and real hardware present as literally "the
  same camera" to everything downstream, not just matched topic names),
  OpenVINS (`ov_msckf`) runs mono VIO against that feed, and
  `openvins_odometry_bridge` converts its output (ENU/FLU → NED/FRD) onto
  the same target topic. Requires `make build` (§4.2) to have built OpenVINS
  — see below — and `PX4_GZ_WORLD=vio_test` at sim-start time (§4.3).

### Building the VIO pipeline (`VIO_BACKEND=openvins`)

OpenVINS and a Gazebo-Harmonic-matched `ros_gz_bridge` are baked into the
`ros2-autonomy` image (`docker/Dockerfile.ros2_autonomy`) — `make build`
(§4.2) builds them along with everything else; there is no separate install
step. Two non-obvious things worth knowing if you ever touch that
Dockerfile:

- **Humble's default `ros-humble-ros-gz-bridge` (from packages.ros.org) is
  the wrong package** — it's built against Gazebo *Fortress*
  (`libignition-transport11`), not the Harmonic
  (`libgz-transport13`) that `px4-sim` actually runs. It would install and
  run with no error, then silently receive zero messages — a genuinely hard
  failure to diagnose. The correct package is OSRF's own
  `ros-humble-ros-gzharmonic-bridge`, from the same
  `packages.osrfoundation.org` apt repo `Dockerfile.px4_sim` already uses.
- OpenVINS has no Humble apt package — it's built from source
  (`github.com/rpng/open_vins`) into its own underlay workspace
  (`/opt/openvins_ws`, same pattern as `/opt/px4_ros2_ws`), with its ROS 2
  dependencies resolved via `rosdep` rather than a hand-maintained list.

Full derivation of the camera intrinsics, the camera-IMU extrinsic
calibration, and every other Milestone B design decision — including
several real bugs found only by actually flight-testing rather than
assuming a design was correct (§12, issues 17-25) — is in
`resource/phase3-gps-denied-localization-source.md`.

`common_control`/`common_missions` are completely unaware of any of this —
they only ever read PX4's already-fused `vehicle_local_position_v1`.

---

## 9. Launch Files Reference

| File | Package | Arguments | Example |
|---|---|---|---|
| `sim.launch.py` | `sim_bringup` | `action` (`hover`\|`mission`, default `mission`) · `mission` (`square`, default `square` — currently the only mission) · `localization_source` (`gps`\|`vision`, default `gps`) · `vio_backend` (`loopback`\|`openvins`, default `loopback`, only used when `localization_source:=vision`) · `mavlink_url` (default `udpin:0.0.0.0:14540`) | `ros2 launch sim_bringup sim.launch.py action:=mission mission:=square localization_source:=vision vio_backend:=openvins` |
| `hw.launch.py` *(Phase 4 stub, untested)* | `hw_bringup` | `serial_device` (default `/dev/ttyUSB0`) · `baud` (default `921600`) · `action` · `mission` · `localization_source` · `vio_backend` · `mavlink_url` (UNTESTED SITL-shaped default) | `ros2 launch hw_bringup hw.launch.py serial_device:=/dev/ttyACM0 action:=mission mission:=square` |
| `autonomy.launch.py` | `common_missions` | `action` (`hover`\|`mission`) · `mission` (`square`) · `params_file` (path, optional) | `ros2 launch common_missions autonomy.launch.py action:=hover` |
| `mission.launch.py` | `common_missions` | `mission` (`square`, default `square`) | `ros2 launch common_missions mission.launch.py mission:=square` |

`sim.launch.py` and `hw.launch.py` are what `make mission`/`make
flight-test` actually run (§5) — they include `autonomy.launch.py` and pass
their own `params_file`. That's the entire sim-vs-real difference at the
launch layer: same `autonomy.launch.py`, different params file and DDS
transport. `mission.launch.py` is the standalone, transport-agnostic mission
selector `autonomy.launch.py` itself builds on; call it directly if you want
a mission with zero params-file involvement.

---

## 10. Environment Variables Reference

Set in `.env` at the repo root, passed through by `docker-compose.yml` as
build args and/or runtime environment. Edit `.env` to change any of these —
no `make` parameter needed. **Never add an inline `# comment` after a
value** — see §12 issue 12.

| Variable | Default | Purpose |
|---|---|---|
| `PX4_VERSION` | `v1.17.0` | PX4 firmware tag cloned into `px4-sim`. Must match `PX4_MSGS_BRANCH`/`PX4_ROS_COM_BRANCH`. Flash this same version to the real Pixhawk 6 in Phase 4. |
| `PX4_MSGS_BRANCH` | `release/1.17` | `px4_msgs` branch built into `ros2-autonomy`. |
| `PX4_ROS_COM_BRANCH` | `release/1.16` | `px4_ros_com` branch built into `ros2-autonomy`. |
| `ROS_DISTRO` | `humble` | ROS 2 distribution. |
| `PX4_GZ_MODEL` | `x500_depth` | Gazebo vehicle model (x500 quad + simulated D435i — `docker/px4_sitl_models/d435i`, §11). |
| `PX4_GZ_WORLD` | *(deliberately unset — see below)* | Gazebo world by name: `empty` (default, bare ground) or `vio_test` (required for `VIO_BACKEND=openvins`, §4.3/§8); any world PX4 bundles also works (`default`, `walls`, `baylands`, …). **Set this on the command line only** (`PX4_GZ_WORLD=vio_test make sim-gui`) — **do not add a `PX4_GZ_WORLD=...` line to `.env`**, it will silently defeat the command-line override (§12 issue 19), which is exactly why this row has no default value here. |
| `PX4_HEADLESS` | `1` | `1` = headless server-only Gazebo. Don't hand-edit this for GUI mode — use `make sim-gui`, which sets it via a compose overlay. |
| `UXRCE_DDS_PORT` | `8888` | Micro-XRCE-DDS-Agent UDP port (PX4 ↔ ROS 2 bridge). |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | ROS 2 middleware implementation. |
| `DRONE_NAMESPACE` | `drone1` | Reserved for future multi-vehicle namespacing; not yet consumed by any node. |
| `LIBREALSENSE_VERSION` | `2.58.2` | librealsense SDK version — must match the physical D435i firmware (Phase 4). |
| `PIXHAWK_SERIAL_PORT` | `/dev/ttyUSB0` | Serial device for the real Pixhawk 6 (Phase 4; matches `hw_bringup`'s `serial_device` default). |
| `PIXHAWK_BAUD_RATE` | `921600` | Serial baud rate for the real Pixhawk 6 (Phase 4). |

---

## 11. No Upstream Repos Are Forked or Modified

Everything external — PX4-Autopilot, px4_msgs, px4_ros_com,
Micro-XRCE-DDS-Agent, OpenVINS — is cloned **read-only at pinned versions
from the official public repos during `docker build`**. All customization
lives as small overlay files inside *this* repo, `COPY`'d into the image
*after* the clone step, at build time — the same mechanism every time,
whether it's a PX4 param override, a world, or a whole Gazebo model:

| What | Overlay file(s) | Overwrites/adds |
|---|---|---|
| PX4 arming param override | `docker/px4_sitl_overrides/4002_gz_x500_depth.post` | A SITL-only airframe hook (§12 issue 4) |
| Gazebo worlds | `docker/px4_sitl_worlds/empty.sdf`, `vio_test.sdf` | `Tools/simulation/gz/worlds/` |
| Gazebo camera model | `docker/px4_sitl_models/d435i/` (new model), `docker/px4_sitl_models/x500_d435i_depth/model.sdf` (swaps the stock `x500_depth`'s camera from a generic OakD-Lite to this repo's own D435i model) | `Tools/simulation/gz/models/` |

Nothing to fork, nothing to patch by hand, no upstream maintenance burden: to
replicate the system anywhere, this repo + Docker is the complete recipe.

---

## 12. Known Issues Hit During Bring-Up

Documenting these so a future rebuild-from-scratch doesn't waste time
rediscovering them — all are already fixed in this repo.

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

**4. PX4 refuses to arm the `x500_depth` SITL model out of the box.** Two
separate prearm checks fail forever with a bare
`WARN [commander] Arming denied: Resolve system health failures first`:
- The x500_depth Gazebo model has no battery/power simulation, so
  `battery_status` never publishes and the battery + power checks never pass.
- The x500 airframe config sets `NAV_DLL_ACT 2` (datalink-loss failsafe),
  and PX4's `rcAndDataLinkCheck` makes a ground-station connection
  **mandatory for arming** whenever that param is > 0 — but this stack flies
  offboard with no QGroundControl attached.

Fixed via `docker/px4_sitl_overrides/4002_gz_x500_depth.post`, which PX4
sources automatically after the airframe config (its own supported override
hook): `CBRK_SUPPLY_CHK 894281` (PX4's documented circuit breaker for
missing power telemetry) and `NAV_DLL_ACT 0` (PX4's code default). Both are
**SITL-only** — the file only exists inside the sim image and is keyed to a
simulation-only airframe ID, so it can never leak onto the real Pixhawk.
Debugging tip that cracked this: `px4-listener health_report` exposes
`arming_check_error_flags` as a bitmask, decodable against
`health_component_t` in `build/px4_sitl_default/events/common_with_enums.json`.
Note the `.post` file must also be copied into the already-compiled
`build/px4_sitl_default/etc/` tree if PX4 was compiled earlier in the same
Dockerfile — the build step snapshots ROMFS at compile time.

**5. Restarting only the px4-sim container wedges the DDS bridge.** If
px4-sim is recreated while ros2-autonomy (and its Micro-XRCE-DDS-Agent) keeps
running, the session "re-establishes" in the agent log and datawriters get
recreated — but no data flows, and `px4-uxrce_dds_client status` inside PX4
reports "Running, disconnected" with timesync never converging. Restarting
the agent process alone did not recover it either. **Workaround: always
restart the full stack together** (`make stop && make sim`).

**6. PX4 v1.17 publishes versioned topic names.** Messages that carry a
`MESSAGE_VERSION` field (e.g. `VehicleStatus`, `VehicleLocalPosition`) appear
on the wire as `/fmu/out/vehicle_status_v1`, `/fmu/out/vehicle_local_position_v1`
— not the unversioned names used in most PX4 examples and docs. Subscribing
to the unversioned name silently receives nothing. `common_control` already
uses the `_v1` names.

**7. The offboard signal = heartbeat + setpoint stream, and commands need
retries.** PX4 only treats the offboard link as "present" when both the
`OffboardControlMode` heartbeat *and* an actual `TrajectorySetpoint` stream
are flowing — a node that waits to publish setpoints until after arming will
never be allowed to arm. Additionally, the first offboard-mode/arm command
right after boot is often rejected while preflight checks settle.
`offboard_control_node` therefore streams the takeoff setpoint from tick 0
(harmless while disarmed on the ground) and retries mode-switch + arm once
per second until `vehicle_status` confirms — never fire-and-forget.

**8. Arming is rejected for the first ~30–60 s after `make sim` — this is
normal.** PX4's EKF2 estimator needs time to converge after boot; until then
the log shows `Preflight Fail: ekf2 missing data` and
`pre_flight_checks_pass: false`. Do not debug this — just wait.
`make flight-test` handles it automatically (the node retries once per
second until PX4 accepts). To watch convergence yourself:
`make shell` → `ros2 topic echo /fmu/out/vehicle_status_v1 --once` and wait
for `pre_flight_checks_pass: true`.

**9. On a weak/older CPU the simulation runs slower than wall-clock.** PX4
SITL runs in lockstep with Gazebo: when the CPU can't keep up, simulated
time simply advances slower than real time — sim/wall-clock speed is not
fixed and varies run to run (a 2 m climb has taken anywhere from ~13 s to
~77 s on the same dev laptop across sessions). Nothing is hung; the flight is
proceeding correctly in sim-time. Judge progress by the node's state
transitions (climbing → hovering → landing), never by a stopwatch, and be
generous with any `timeout` you wrap around the test.

**10. If the first `make build` dies mid-way (network hiccup), just rerun
it.** The px4-sim image clones PX4 + all submodules and downloads Gazebo
Harmonic packages — several GB total. Docker caches each completed layer,
so a rerun resumes from the last finished step instead of starting over.

**11. Both containers use host networking — check for conflicts on shared
machines.** The Micro-XRCE-DDS-Agent binds UDP `8888` on the host; PX4's
MAVLink uses `14550`/`14540`. If another process holds those ports, the
bridge silently fails. Likewise, ROS 2 traffic uses the default
`ROS_DOMAIN_ID=0` — if other ROS 2 systems run on the same host/LAN, their
topics will cross-talk; set a unique `ROS_DOMAIN_ID` for this stack (add it
to the `environment:` of `ros2-autonomy` in `docker-compose.yml`) if that
applies to you.

**12. Never put an inline `# comment` after a value in `.env`.** docker
compose strips the comment itself but keeps the whitespace padding before it
as part of the value — `UXRCE_DDS_PORT=8888   # comment` becomes the string
`"8888   "`. MicroXRCEAgent rejects the padded port and exits immediately at
container boot, so the PX4 ↔ ROS 2 bridge never comes up: every `/fmu/*`
topic exists but is silent, and any control node waits forever with
`nav_state=0, arming_state=0`. Diagnose with
`docker exec ros2-autonomy cat /var/log/microxrce_agent.log` (shows a
`'--port <value>' is required` usage error) — fixed by keeping `.env`
comments on their own lines, and the entrypoint now also trims whitespace
from the port value as a safety net.

**13. `HEADLESS=0` does NOT enable the Gazebo GUI — it must be unset.** PX4's
`px4-rc.gzsim` init script decides whether to launch `gz sim -g` (the GUI
client) with `if [ -z "${HEADLESS}" ]` — a check for "unset or empty", not
"falsy". Setting `HEADLESS=0` still satisfies "set", so the GUI silently
never starts and you just get the headless server with no error. Fixed in
`docker/Dockerfile.px4_sim`'s `CMD`: it branches on `PX4_HEADLESS` and either
`export HEADLESS=1` or `unset HEADLESS` before invoking `make px4_sitl
gz_<model>` — never `HEADLESS=0`. `make sim-gui` sets `PX4_HEADLESS=0` (a
compose env var, distinct from PX4's own `HEADLESS`) precisely so the CMD's
branch can translate it into an *unset* `HEADLESS` for PX4.

**14. PX4's uXRCE-DDS bridge cannot set PX4 parameters at all.** Every other
command in this repo talks to PX4 over its ROS 2 / uXRCE-DDS bridge, but
that bridge carries no `Parameter*` topic in this pinned version (confirmed
by exhaustively grepping PX4's `dds_topics.yaml`) — so switching PX4's
`EKF2_GPS_CTRL`/`EKF2_EV_CTRL` needed a separate MAVLink `PARAM_SET`
side-channel (`common_perception`'s `set_localization_source`). See §8 and
`resource/phase3-gps-denied-localization-source.md` for the full mechanism.

**15. Humble's default `ros-humble-ros-gz-bridge` targets the wrong Gazebo
version.** It's built against Gazebo *Fortress* (`libignition-transport11`),
confirmed via `apt-cache depends` — but `px4-sim` runs Gazebo *Harmonic*
(`libgz-transport13`), a different, incompatible wire protocol. Installing
the default package builds and runs with **zero errors**, then silently
receives **zero messages** from Gazebo — a config that looks entirely
correct while doing nothing. Fixed by using OSRF's own
`ros-humble-ros-gzharmonic-bridge` instead (same
`packages.osrfoundation.org` repo `Dockerfile.px4_sim` already trusts for
Gazebo itself), verified to depend on `libgz-transport13` before adopting it.

**16. A `RUN ... && rm -rf /var/lib/apt/lists/*` layer wipes the apt cache
for every subsequent `RUN` layer, not just that one.** A later layer's
`apt-get install` (triggered internally by `rosdep install` when building
OpenVINS) failed with `Unable to locate package ros-humble-cv-bridge`
because an earlier layer's cleanup had already removed the package index,
and `rosdep update` (rosdep's own dependency-key database) does **not**
imply `apt-get update` (apt's package index) — two unrelated caches. Fixed
by adding an explicit `apt-get update` at the start of that later layer.

**17. Real VIO's cruise speed matters more than it looks — too slow starves
scale/bias observability.** `square`/`survey` originally flew at
`max_velocity_m_s: 0.2` under `VIO_BACKEND=openvins`. A landing-phase
divergence (issue 21) was traced, in part, by grepping OpenVINS's own debug
log across the whole flight: accelerometer bias was still actively
*growing*, never converged, by landing time (`-0.0066 → -0.0318 →
-0.0174,0.0428`) — vs. converged and stable in an earlier flight at
`1.0 m/s`. Monocular VIO can only observe scale/IMU bias from real
acceleration events; a long, near-constant-velocity flight starves it of
exactly that for the whole mission. Fixed by raising missions to `0.8 m/s`
(§7). Don't set this back below ~0.5 for `openvins` flights.

**18. `EKF2_EV_POS_X/Y/Z` (the vision sensor's lever arm) was never
configured — a genuine, previously-missing bug, not a hypothetical.**
Confirmed via `grep`: nothing in this repo ever set it. The published
vision odometry represents the simulated D435i's own onboard IMU, offset
from the flight controller's IMU by the camera's mount pose — without
telling EKF2 that offset, it assumes they're the same point, and any
attitude change (pitch/roll) makes the offset sensor translate through
space in a way the vehicle's true center doesn't, which EKF2 misattributes
as genuine motion. Landing/correction maneuvers have more attitude change
than steady cruise, so this bites hardest exactly there. Fixed:
`set_localization_source.py` now sets this automatically on
`--source vision` (§8).

**19. Setting `PX4_GZ_WORLD` in `.env` silently defeats the
`PX4_GZ_WORLD=vio_test make sim-gui` command-line override — a real,
reproduced bug.** The Makefile's `include .env` + `export` means a plain
`VAR=value` line in `.env` is a genuine Makefile variable assignment, and
GNU Make always lets a Makefile-internal assignment win over a same-named
shell/environment variable — regardless of the Makefile's own
`PX4_GZ_WORLD ?= empty` default, which exists specifically to allow that
override. `.env` used to hardcode `PX4_GZ_WORLD=empty`, so
`PX4_GZ_WORLD=vio_test make sim-gui` silently ran the `empty` world anyway,
with no error. Fixed by removing that line from `.env` entirely — it's a
per-invocation runtime choice (like `MISSION=`/`LOCALIZATION=`), not a
build-time constant like the version pins around it, so it doesn't belong
in `.env` at all. Always pass `PX4_GZ_WORLD` on the command line.

**20. The drone can be missing from the Gazebo GUI's 3D view/entity tree
even though it's simulating completely correctly.** The world's static
content (ground, props) renders fine because it's in the GUI client's
initial scene load; the vehicle is spawned dynamically by PX4 *after* the
world/GUI are already up, and a client that connects right around that
moment can miss the "new entity" notification. Confirmed this is a
GUI-rendering issue, not a simulation bug, via `gz service
-s /world/<world>/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene`
(returns the vehicle's full mesh/pose data even when the GUI shows
nothing) and `gz topic -e -t /world/<world>/pose/info` (shows a live,
correct pose). **The fix that actually works is `make gz-resync`, which
restarts only the GUI client process** — a pure viewer; the sim server,
PX4, and any in-progress flight are untouched — so it reconnects and
downloads the complete scene fresh. Two things that do NOT work, both
tried live: (a) calling the `scene/info` *service* from the CLI — its
response goes to the caller, not the GUI, so an already-desynced client
learns nothing from it (an earlier version of `make gz-resync` did this
and was confirmed ineffective); (b) waiting — a desynced client never
catches up on its own. Note the restart must re-export
`GZ_SIM_RESOURCE_PATH` and `GZ_IP` (the target handles this): those are
set inside PX4's launch-script session, not the container's top-level
environment, and without the resource path the fresh GUI would resolve
none of the drone's `model://` mesh URIs — same invisible-drone symptom,
different cause. Two load-related contributing factors were also fixed/
identified: PX4's `pxh>` shell, given no TTY (the `docker compose up -d`
default), spins retrying reads on closed stdin — fixed with `tty: true` +
`stdin_open: true` on `px4-sim` in `docker-compose.yml` (confirmed live:
`pxh>` prompt-spam in the logs disappeared and `px4`'s CPU dropped
substantially) — and the `vio_test` scene's 100+ static prop/tile entities
(issue 23) plus the simulated camera sensors are genuine rendering load on
an older host: a real-time factor well under 1.0 (`gz topic -e -t
/world/<world>/stats`) is expected on modest hardware (issue 9), and the
slower the host, the more likely the GUI's initial sync loses this race —
so expect to need `make gz-resync` occasionally on such machines.

**21. Post-landing, PX4's own position/velocity ESTIMATE can keep drifting
even though the vehicle is physically at rest — confirmed via Gazebo's own
ground-truth pose, independent of the estimate — which blocks PX4's normal
auto-disarm forever.** Two fixes were tried INSIDE OpenVINS and both
reverted after live failures, not just reasoned away: enabling OpenVINS's
own zero-velocity update (ZUPT) throughout flight (not just pre-arm)
self-reinforcing-locks the estimate to "stationary" at the exact instant
every takeoff begins (v=0 there, by definition) — confirmed live, the
vehicle never left the ground. A hand-rolled raw-IMU "stillness override"
in the odometry bridge failed the same way for a related reason:
accelerometer-based "is moving" detection can't see constant-velocity
motion, only acceleration, and this project's deliberately slow missions
(issue 17) look "stationary" to that kind of threshold for their whole
flight. **Fixed one level up instead**, in `common_control` — see
`land_disarm_low_throttle_dwell_s`/`land_disarm_max_timeout_s` (§7): disarm
once PX4's own actuator thrust (`vehicle_land_detected.has_low_throttle` —
a pure control-output signal, confirmed via PX4's own
`MulticopterLandDetector.cpp` source to be independent of the drifting
position/velocity estimate) stays sustained-low, never based on elapsed
time alone. One more real bug caught in the process: a plain disarm command
is silently `MAV_RESULT_TEMPORARILY_REJECTED` by PX4 unless
`vehicle_land_detected.landed` is already true OR the command's `param2` is
PX4's documented force-arm/disarm magic value `21196` — without that,
this whole fallback would have been a silent no-op. Confirmed clean on
repeated full mission retests afterward.

**22. Wrong VIO camera-IMU extrinsic rotation caused a real in-flight
divergence — twice, for two different reasons.** First: `T_cam_imu`'s
rotation was identity — wrong, because a camera's declared mount pose and
its actual pixel-projection ("optical") frame are different frames
regardless of translation, for any forward-facing camera. Second, after
fixing the rotation itself: the DIRECTION was backwards (a transpose bug).
OpenVINS's own online extrinsics calibration silently self-corrected the
second bug during smooth cruise flight, masking it — but not during
`AUTO_LAND`'s more aggressive dynamics, where the filter genuinely
diverged and PX4's own landing controller, trusting that state, flew the
real vehicle off course (confirmed via Gazebo ground truth, not the
estimate). Fixed by seeding the correct, empirically-confirmed rotation
directly (`config/openvins/kalibr_imucam_chain.yaml`) and freezing it
(`calib_cam_extrinsics: false`) so there's no more online value to get
backwards. Full derivation:
`resource/phase3-gps-denied-localization-source.md`.

**23. Monocular VIO lost all trackable features near the ground —
`vio_test.sdf`'s ground plane was flat, uniform gray.** The "fence" props
are tall (0.35–1.6 m); at low altitude near the landing point their tops
exit the D435i's ~42° vertical FOV, leaving only the flat ground plane in
view — zero texture gradient, zero corners for the KLT tracker, right when
the vehicle is landing and precision matters most. Fixed by adding a
64-tile ground-level checkerboard (alternating light/dark 1×1 m boxes,
same technique as the fence props — no external texture/material asset,
no missing-asset risk) covering the whole mission footprint. Confirmed
live: OpenVINS tracked 42–44 features pre-arm afterward, up from ~27
before.

**24. A `square` mission flew out of the fenced/textured area at speed and
hit the ground — a real crash, reported 2026-07-09, not reproduced on the
very next clean run (this stack's VIO divergence is stochastic, not
deterministic — see issue 22; a config can run clean several times then
diverge with nothing changed).** Rather than keep chasing a specific
trigger, added a defense-in-depth geofence in `offboard_control_node.py`
independent of root cause: bounds are auto-derived from the current route's
own bounding box (origin + every queued waypoint) plus
`geofence_margin_m`/`geofence_height_margin_m` (§7) — "modular" per the
original ask, meaning it automatically follows whatever a mission's own
`side_length_m`/`area_length_m`+`area_width_m`/etc. describe, with no
mission-type-specific code. A breach in any flying state aborts immediately
to `LAND` — same AUTO_LAND path a normal mission end uses, backed by the
existing estimate-independent disarm fallback (issue 21). This can't
distinguish a genuinely-diverged estimate from a real fence-crossing (both
read the same way from the only position source this architecture has) —
that's an accepted tradeoff: a false-positive early landing is a much
cheaper failure than the crash it replaces.

**25. While reproducing issue 24, found a second, worse bug: a
flight/mission node's OS process can outlive its own "mission complete" —
`rclpy.shutdown()` doesn't reliably unblock `rclpy.spin()` here (this
project's `rmw_cyclonedds_cpp` RMW leaves non-daemon background threads a
plain interpreter exit won't wait past).** Confirmed live: a leftover
`square_mission` process kept running for minutes after logging "Landed and
disarmed — mission complete," and the OpenVINS instance sharing its launch
— with nothing left to sanity-check it against — ran its own position
estimate away to 100+ meters, distance-traveled 150 m, well past this
`README`'s existing "estimate can drift post-landing" caveat (issue 21).
Worse, PX4's own EKF2 fused enough of that runaway before disarm to leave
`vehicle_local_position` reporting tens of meters off, which then silently
broke arming for the *next* flight in the same containers (another
real-looking "it doesn't work" that's actually this). Two fixes, together:
(a) `offboard_control_node.py`'s `FlightState.DONE` now calls `os._exit(0)`
directly — force real OS-level process termination the instant landing +
disarm are confirmed, rather than trust `rclpy.spin()` to return control
back up to `main()`'s own cleanup path. It doesn't: confirmed live, calling
`rclpy.shutdown()` from a timer callback running on the executor's own spin
thread can leave that call never returning, so nothing after it (including
an `os._exit(0)` placed just one line later) ever runs either — `DONE` now
skips `rclpy.shutdown()` entirely, since a hard process exit makes graceful
rclpy teardown moot anyway. (b) `autonomy.launch.py` now registers an
`on_exit=Shutdown()` handler on the flight/mission node so the whole
launch — including sibling VIO/bridge nodes, which have no exit condition
of their own — tears down the moment that node's process actually exits,
success or failure. Confirmed live: "mission complete" immediately followed
by the process exiting cleanly, the launch shutting the rest of the system
down, and zero leftover processes afterward. Operational takeaway either
way: after any flight, a
full `make stop` (or letting `make mission`/`make flight-test` run to
completion in its own terminal rather than backgrounding it) is still the
safest habit — don't trust a "mission complete" log line alone to mean the
containers are idle.

**26. Manually launching an image viewer (`rqt_image_view`) per mission is
a sequencing race, not just extra typing — a viewer started after the
mission has already begun (or, worse, already finished — see issue 25's
`on_exit=Shutdown()`) can lose the race and never see a single frame,
regardless of which viewer or how long you wait.** Fixed by making the
viewer part of the stack's own lifecycle instead of a manually-timed
afterthought: `docker-compose.gui.yml` now starts a third container,
`rqt-viewer`, alongside `px4-sim`/`ros2-autonomy` under `make sim-gui` —
subscribed to `/camera/camera/color/image_raw` from the moment the stack
comes up, before any mission's camera bridge exists. DDS discovery
connects a subscriber to a not-yet-existing publisher automatically the
moment that publisher appears, so ordering after that point stops
mattering. `ros-humble-rqt-image-view` is baked into the `ros2-autonomy`
image (`Dockerfile.ros2_autonomy`) rather than `apt-get install`ed at
container-start time, so the viewer opens immediately rather than after a
30-60s package download on every single stack restart. One real gotcha
worth knowing if extending this pattern: the new `rqt-viewer` service
reuses the `ros2-autonomy` image and therefore its `ENTRYPOINT`, which
starts a second `Micro-XRCE-DDS-Agent` — since `rqt-viewer` shares the host
network namespace (`network_mode: host`) with `ros2-autonomy`, which
already runs one bound to the same UDP port, a second instance would
either fail to bind or fight over the same PX4 DDS stream. `rqt-viewer`
overrides `entrypoint: []` to skip this — it only needs `rqt_image_view`,
not a DDS agent of its own. `make stop` was updated to always tear down
both compose files together (`--remove-orphans`), so this container gets
cleaned up whether you used `make sim` or `make sim-gui` last.

---

## 13. Repository Layout

```
docker/
  Dockerfile.px4_sim              PX4 v1.17.0 SITL + Gazebo Harmonic (headless default, GUI opt-in)
  Dockerfile.ros2_autonomy        ROS 2 Humble + uXRCE-DDS agent + px4_msgs/px4_ros_com + OpenVINS
  entrypoint_ros2_autonomy.sh     Auto-starts the Micro-XRCE-DDS-Agent at container boot
  px4_sitl_overrides/             SITL-only PX4 param overrides (see §12, issue 4)
  px4_sitl_worlds/                Overlay Gazebo worlds (empty.sdf, vio_test.sdf — §4.3/§8)
  px4_sitl_models/                Overlay Gazebo model: this repo's own D435i camera (§11)
docker-compose.yml                sim profile: px4-sim + ros2-autonomy (host networking)
docker-compose.gui.yml            Opt-in overlay: X11/DRI passthrough for `make sim-gui`
.env                              Version pins + runtime config (§10)
Makefile                          build / sim / sim-gui / flight-test / mission / gz-resync / stop / shell
resource/
  phase3-gps-denied-localization-source.md   Full VIO/localization-switch design + debugging history
ros2_ws/src/
  common_control/                 OffboardControlNode — heartbeat/arm/offboard/
                                   takeoff/waypoints/hover/land state machine, including
                                   the post-landing disarm fallback (§7, §12 issue 21) (Phase 1 ✓)
  common_missions/                MissionBase + the square mission + the shared,
                                   transport-agnostic autonomy.launch.py (Phase 2 ✓)
  common_perception/               Localization-source switch (GPS/vision, §8) + both VIO
                                   backends (loopback stand-in, real OpenVINS) (Phase 3 ✓)
  sim_bringup/                    Sim-only launch + params (sim_params.yaml) — includes
                                   autonomy.launch.py, no flight logic of its own (Phase 2.5 ✓)
  hw_bringup/                     Real-hardware bringup STUB — serial uXRCE-DDS agent +
                                   hw_params.yaml; untested, no hardware yet (Phase 4 seam)
```

---

## 14. Roadmap

This repo is under active development.

- **Phase 0 — complete**: PX4 SITL + Gazebo Harmonic boots headless; the
  uXRCE-DDS bridge exposes the full `/fmu/*` topic set to ROS 2.
- **Phase 1 — complete**: `common_control/offboard_control_node` flies the
  full cycle in sim — arm → offboard → takeoff → hover → land → disarm
  (`make flight-test`).
- **Phase 2 — complete**: pluggable missions (`common_missions`, selected by
  name via `make mission MISSION=<name>` — currently just `square`; a
  `survey` lawnmower mission existed and flew successfully but was removed
  2026-07-09 pending a purpose-built world for its footprint) on top of the
  Phase 1 control primitives.
- **Phase 2.5 — complete**: `sim_bringup`/`hw_bringup` packages formalize the
  sim/real split (launch + params only, never flight logic — see §2 Rule),
  plus an opt-in Gazebo GUI (`make sim-gui`) to watch flights visually.
  `hw_bringup` ships as an untested stub (serial DDS agent + param file) —
  real hardware wiring is still Phase 4. Every tuning value (takeoff height,
  mission geometry, …) is a required parameter with no code-side default —
  it must come from `sim_params.yaml`/`hw_params.yaml` (§7) or startup fails
  loudly — and `make build-ws` is split from `make flight-test`/`make
  mission` so editing that YAML never triggers a rebuild.
- **Phase 3 — complete**: GPS-denied flight in sim (§8). Milestone A
  (`VIO_BACKEND=loopback`, a zero-drift fake-VIO stand-in) proved the
  GPS/vision switch mechanism end to end. Milestone B (`VIO_BACKEND=openvins`,
  real monocular VIO) flies both missions GPS-denied, including landing and
  auto-disarm, confirmed on repeated clean retests after fixing several real
  bugs found via actual flight testing (§12, issues 17-25) — camera
  extrinsics (wrong, then wrong direction), a missing EKF2 sensor lever arm,
  IMU-excitation starvation from too-slow cruise speeds, visual feature
  starvation near the ground, and a post-landing disarm gap. Treat as a
  strong, well-evidenced fix rather than an absolute guarantee — this
  project's own testing saw a config fly clean multiple times, then diverge,
  with nothing changed, before the real root causes were found. Full
  incident history: `resource/phase3-gps-denied-localization-source.md`.
- **Phase 4**: same code on real hardware — Orange Pi 5 Plus + Pixhawk 6C +
  RealSense D435i. Not started; `hw_bringup` remains an untested stub.
- **Phase 5**: SLAM + Nav2 navigation.

Full phase-by-phase detail lives in `IMPLEMENTATION_PLAN.md` (local,
gitignored, not pushed — an internal working document that changes too fast
to keep in sync with the public repo).
