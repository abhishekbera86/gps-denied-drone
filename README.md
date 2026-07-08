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
8. [Launch Files Reference](#8-launch-files-reference)
9. [Environment Variables Reference](#9-environment-variables-reference)
10. [No Upstream Repos Are Forked or Modified](#10-no-upstream-repos-are-forked-or-modified)
11. [Known Issues Hit During Bring-Up](#11-known-issues-hit-during-bring-up)
12. [Repository Layout](#12-repository-layout)
13. [Roadmap](#13-roadmap)

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
  solid (not yet built — see [Roadmap](#13-roadmap))

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
        │  GPU optional     │            │                     │
        │  (GUI opt-in)     │            │  STUB — untested,   │
        │  px4-sim:         │            │  no hardware yet.   │
        │   PX4 v1.17.0 SITL│            │  Serial uXRCE-DDS   │
        │   + Gazebo Harmonic│           │  agent + realsense/ │
        │   headless or GUI │            │  perception slots — │
        │                   │            │  see §13 Roadmap    │
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
[§10](#10-no-upstream-repos-are-forked-or-modified).

### 4.2 Build the Docker Images

```bash
make build
```

Compiles two images (expect ~15–40 min on the very first run, depending on
CPU and network — later rebuilds hit Docker layer cache and take seconds):

- **`px4-sim`** — clones PX4-Autopilot at the pinned `v1.17.0` tag, installs
  Gazebo Harmonic via PX4's own `Tools/setup/ubuntu.sh`, pre-compiles the
  `px4_sitl_default` target, and overlays this repo's SITL-only param file
  (§11 issue 4) and empty world (§11 issue 13).
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

The world is `docker/px4_sitl_worlds/empty.sdf` — a bare ground plane, sun,
and grid (an overlay file, not a fork of PX4; same mechanism as the SITL
param override, §11 issue 4) — open space to watch takeoff and both missions
without scenery in the way.

If the window doesn't appear, or Gazebo crashes trying to use your GPU, force
software rendering instead:
```bash
GZ_SW_RENDER=1 make sim-gui
```

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

Colcon-builds `common_control`, `common_missions`, `sim_bringup`, and
`hw_bringup` inside `ros2-autonomy` with `--symlink-install`, into the
persistent `/ros2_ws_build` volume. Source is live-mounted from
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
wrong with this repo (§11 issue 8). You don't have to do anything: every
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
3. Takes off to 2 m, hovers 5 s, lands, and PX4 auto-disarms on touchdown

Expected output ends with:
```
[INFO] [...]: Armed and offboard — climbing to takeoff height
[INFO] [...]: Reached takeoff height — hovering for 5.0s
[INFO] [...]: Hover complete — landing
[INFO] [...]: Landed and disarmed — mission complete
```

### 4.7 Fly a Mission

```bash
make mission                    # square, the default
make mission MISSION=survey     # lawnmower coverage pattern
```

Runs `ros2 launch sim_bringup sim.launch.py action:=mission
mission:=<name>`. Two missions exist today:

- **`square`** (default) — 4 m square at takeoff height, nose pointed along
  each leg, landing back at the start.
- **`survey`** — lawnmower coverage of an 8 m × 6 m rectangle with 2 m lane
  spacing, returning to the start to land.

Both are subclasses of `MissionBase`, which reuses the entire hover-test
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
the DDS bridge wedged — see §11 issue 5.

---

## 5. Command Reference

Every `make` target, what it does, and any parameters it accepts. Parameters
are passed as `VAR=value` after the target name (standard `make` override
syntax) or as an environment variable before it.

| Command | Parameters | What it does |
|---|---|---|
| `make build` | — | Builds the `px4-sim` and `ros2-autonomy` **Docker images**. One-time, or after editing a Dockerfile. |
| `make build-ws` | — | Colcon-builds the **ROS 2 workspace** inside `ros2-autonomy`. One-time, or after adding/removing a package or editing `setup.py`/`package.xml` — NOT needed after editing Python/launch/config files (§4.4). |
| `make sim` | — | Starts PX4 SITL (Gazebo Harmonic, **headless**) + the ROS 2 bridge container. |
| `make sim-gui` | `GZ_SW_RENDER=1` *(optional)* — forces software (llvmpipe) rendering if the GPU path fails | Same stack as `make sim`, but with the real Gazebo GUI window forwarded to the host X display. |
| `make flight-test` | — | Flies the hover test (arm → takeoff → hover → land) via `sim_bringup`. Requires `make build-ws` first. |
| `make mission` | `MISSION=square` *(default)* or `MISSION=survey` | Flies the named waypoint mission via `sim_bringup`. Requires `make build-ws` first. |
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
or subscribes to, over the PX4 uXRCE-DDS bridge. (PX4 exposes many more
`/fmu/out/*` topics than this — odometry, IMU, GPS, battery, and so on — all
visible via `ros2 topic list`; none of them are consumed yet. They're the
natural extension point for `common_perception` in Phase 3.)

| Topic | Direction | Message Type | Rate | Purpose |
|---|---|---|---|---|
| `/fmu/in/offboard_control_mode` | Publish | `px4_msgs/msg/OffboardControlMode` | 10 Hz (`control_rate_hz`) | Heartbeat telling PX4 "position control, offboard, still here" — required continuously or PX4 drops offboard mode. |
| `/fmu/in/trajectory_setpoint` | Publish | `px4_msgs/msg/TrajectorySetpoint` | 10 Hz | The current target `(x, y, z, yaw)` in NED, streamed even while disarmed (§11 issue 7). |
| `/fmu/in/vehicle_command` | Publish | `px4_msgs/msg/VehicleCommand` | On demand | Arm/disarm, offboard mode switch, and `NAV_LAND` commands. |
| `/fmu/out/vehicle_local_position_v1` | Subscribe | `px4_msgs/msg/VehicleLocalPosition` | ~as published by PX4 | Feeds the waypoint-arrival check (`waypoint_tolerance_m` distance test) and the takeoff-height check. |
| `/fmu/out/vehicle_status_v1` | Subscribe | `px4_msgs/msg/VehicleStatus` | ~as published by PX4 | `arming_state` and `nav_state` — drives every state-machine transition (armed? in offboard? disarmed after landing?). |

Notes:
- The `_v1` suffix is not a typo — PX4 v1.17 publishes **versioned** topic
  names for any message carrying a `MESSAGE_VERSION` field. The unversioned
  names used in older PX4 examples receive nothing on this version (§11
  issue 6).
- QoS on every topic above is `BEST_EFFORT` reliability + `TRANSIENT_LOCAL`
  durability + `KEEP_LAST` depth 1 (matching PX4's own DDS QoS) — see
  `PX4_QOS` in `common_control/common_control/offboard_control_node.py` if
  you're writing a new node against this bridge.

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
    max_velocity_m_s: 1.0      # ← lower this if the vehicle moves too fast

square_mission:                # action:=mission mission:=square
  ros__parameters:
    takeoff_height_m: 2.0
    waypoint_tolerance_m: 0.5
    max_velocity_m_s: 1.0
    side_length_m: 4.0         # ← change this, then `make mission MISSION=square`

survey_mission:                # action:=mission mission:=survey
  ros__parameters:
    takeoff_height_m: 2.0
    area_length_m: 8.0
    area_width_m: 6.0
    lane_spacing_m: 2.0
    max_velocity_m_s: 1.0
```

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
| `max_velocity_m_s` | float | `1.0` | `0.5` | Cruise speed cap toward any setpoint (takeoff, hover position, or a waypoint), in m/s. Without this, PX4's own offboard position controller accelerates toward every setpoint at its internal velocity limit — much faster than useful for watching a mission or a first real flight. Implemented as a feed-forward velocity vector aimed at the target, magnitude-capped at this value (`_capped_velocity_toward` in `offboard_control_node.py`) — not a PX4 firmware parameter. |

### `square_mission`

| Parameter | Type | `sim_params.yaml` | `hw_params.yaml` | Meaning |
|---|---|---|---|---|
| `side_length_m` | float | `3.0` | `2.0` | Side length of the square flight path, in meters. |

### `survey_mission`

| Parameter | Type | `sim_params.yaml` | `hw_params.yaml` | Meaning |
|---|---|---|---|---|
| `area_length_m` | float | `5.0` | `4.0` | Length of each lawnmower lane (north axis), in meters. |
| `area_width_m` | float | `5.0` | `3.0` | Total coverage width (east axis) the lanes step across, in meters. |
| `lane_spacing_m` | float | `0.5` | `1.5` | Distance between adjacent lanes, in meters. |

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

## 8. Launch Files Reference

| File | Package | Arguments | Example |
|---|---|---|---|
| `sim.launch.py` | `sim_bringup` | `action` (`hover`\|`mission`, default `mission`) · `mission` (`square`\|`survey`, default `square`) | `ros2 launch sim_bringup sim.launch.py action:=mission mission:=survey` |
| `hw.launch.py` *(Phase 4 stub, untested)* | `hw_bringup` | `serial_device` (default `/dev/ttyUSB0`) · `baud` (default `921600`) · `action` · `mission` | `ros2 launch hw_bringup hw.launch.py serial_device:=/dev/ttyACM0 action:=mission mission:=square` |
| `autonomy.launch.py` | `common_missions` | `action` (`hover`\|`mission`) · `mission` (`square`\|`survey`) · `params_file` (path, optional) | `ros2 launch common_missions autonomy.launch.py action:=hover` |
| `mission.launch.py` | `common_missions` | `mission` (`square`\|`survey`, default `square`) | `ros2 launch common_missions mission.launch.py mission:=survey` |

`sim.launch.py` and `hw.launch.py` are what `make mission`/`make
flight-test` actually run (§5) — they include `autonomy.launch.py` and pass
their own `params_file`. That's the entire sim-vs-real difference at the
launch layer: same `autonomy.launch.py`, different params file and DDS
transport. `mission.launch.py` is the standalone, transport-agnostic mission
selector `autonomy.launch.py` itself builds on; call it directly if you want
a mission with zero params-file involvement.

---

## 9. Environment Variables Reference

Set in `.env` at the repo root, passed through by `docker-compose.yml` as
build args and/or runtime environment. Edit `.env` to change any of these —
no `make` parameter needed. **Never add an inline `# comment` after a
value** — see §11 issue 12.

| Variable | Default | Purpose |
|---|---|---|
| `PX4_VERSION` | `v1.17.0` | PX4 firmware tag cloned into `px4-sim`. Must match `PX4_MSGS_BRANCH`/`PX4_ROS_COM_BRANCH`. Flash this same version to the real Pixhawk 6 in Phase 4. |
| `PX4_MSGS_BRANCH` | `release/1.17` | `px4_msgs` branch built into `ros2-autonomy`. |
| `PX4_ROS_COM_BRANCH` | `release/1.16` | `px4_ros_com` branch built into `ros2-autonomy`. |
| `ROS_DISTRO` | `humble` | ROS 2 distribution. |
| `PX4_GZ_MODEL` | `x500_depth` | Gazebo vehicle model (x500 quad + simulated depth camera, for Phase 3 VIO). |
| `PX4_GZ_WORLD` | `empty` | Gazebo world by name. `empty` is this repo's overlay world (§4.3); any world PX4 bundles also works (`default`, `walls`, `baylands`, …). |
| `PX4_HEADLESS` | `1` | `1` = headless server-only Gazebo. Don't hand-edit this for GUI mode — use `make sim-gui`, which sets it via a compose overlay. |
| `UXRCE_DDS_PORT` | `8888` | Micro-XRCE-DDS-Agent UDP port (PX4 ↔ ROS 2 bridge). |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | ROS 2 middleware implementation. |
| `DRONE_NAMESPACE` | `drone1` | Reserved for future multi-vehicle namespacing; not yet consumed by any node. |
| `LIBREALSENSE_VERSION` | `2.58.2` | librealsense SDK version — must match the physical D435i firmware (Phase 4). |
| `PIXHAWK_SERIAL_PORT` | `/dev/ttyUSB0` | Serial device for the real Pixhawk 6 (Phase 4; matches `hw_bringup`'s `serial_device` default). |
| `PIXHAWK_BAUD_RATE` | `921600` | Serial baud rate for the real Pixhawk 6 (Phase 4). |

---

## 10. No Upstream Repos Are Forked or Modified

Everything external — PX4-Autopilot, px4_msgs, px4_ros_com,
Micro-XRCE-DDS-Agent — is cloned **read-only at pinned versions from the
official public repos during `docker build`**. All customization lives as
small overlay files inside *this* repo (e.g.
`docker/px4_sitl_overrides/4002_gz_x500_depth.post`,
`docker/px4_sitl_worlds/empty.sdf`, copied into the image at build time).
Nothing to fork, nothing to patch by hand, no upstream maintenance burden: to
replicate the system anywhere, this repo + Docker is the complete recipe.

---

## 11. Known Issues Hit During Bring-Up

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

---

## 12. Repository Layout

```
docker/
  Dockerfile.px4_sim              PX4 v1.17.0 SITL + Gazebo Harmonic (headless default, GUI opt-in)
  Dockerfile.ros2_autonomy        ROS 2 Humble + uXRCE-DDS agent + px4_msgs/px4_ros_com
  entrypoint_ros2_autonomy.sh     Auto-starts the Micro-XRCE-DDS-Agent at container boot
  px4_sitl_overrides/             SITL-only PX4 param overrides (see §11, issue 4)
  px4_sitl_worlds/                Overlay Gazebo worlds (empty.sdf — bare ground+sun+grid)
docker-compose.yml                sim profile: px4-sim + ros2-autonomy (host networking)
docker-compose.gui.yml            Opt-in overlay: X11/DRI passthrough for `make sim-gui`
.env                              Version pins + runtime config (§9)
Makefile                          build / sim / sim-gui / flight-test / mission / stop / shell
ros2_ws/src/
  common_control/                 OffboardControlNode — heartbeat/arm/offboard/
                                   takeoff/waypoints/hover/land state machine (Phase 1 ✓)
  common_missions/                MissionBase + square/survey missions + the shared,
                                   transport-agnostic autonomy.launch.py (Phase 2 ✓)
  sim_bringup/                    Sim-only launch + params (sim_params.yaml) — includes
                                   autonomy.launch.py, no flight logic of its own (Phase 2.5 ✓)
  hw_bringup/                     Real-hardware bringup STUB — serial uXRCE-DDS agent +
                                   hw_params.yaml; untested, no hardware yet (Phase 4 seam)
  (common_perception — Phase 3, not built yet)
```

---

## 13. Roadmap

This repo is under active development.

- **Phase 0 — complete**: PX4 SITL + Gazebo Harmonic boots headless; the
  uXRCE-DDS bridge exposes the full `/fmu/*` topic set to ROS 2.
- **Phase 1 — complete**: `common_control/offboard_control_node` flies the
  full cycle in sim — arm → offboard → takeoff → hover → land → disarm
  (`make flight-test`).
- **Phase 2 — complete**: pluggable missions (`common_missions`: square,
  survey, selected by name via `make mission MISSION=<name>`) on top of the
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
- **Phase 3**: GPS-denied flight in sim — OpenVINS VIO fed by the simulated
  depth camera, fused into PX4 EKF2 with GPS disabled.
- **Phase 4**: same code on real hardware — Orange Pi 5 Plus + Pixhawk 6 +
  RealSense D435i.
- **Phase 5**: SLAM + Nav2 navigation.

Full phase-by-phase detail lives in `IMPLEMENTATION_PLAN.md` (local,
gitignored, not pushed — an internal working document that changes too fast
to keep in sync with the public repo).
