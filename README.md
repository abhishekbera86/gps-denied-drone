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

**Status:** Phases 0–3 complete and flight-tested in simulation (GPS and
GPS-denied/VIO). Phase 4 (real hardware) not started. See [Roadmap](#roadmap).

---

## Table of Contents

1. [Objective](#objective)
2. [Architecture](#architecture)
3. [Quick Start](#quick-start)
4. [Documentation](#documentation)
5. [Repository Layout](#repository-layout)
6. [No Upstream Repos Are Forked or Modified](#no-upstream-repos-are-forked-or-modified)
7. [Roadmap](#roadmap)

---

## Objective

- **GPS-denied flight** using VIO from an Intel RealSense D435i
- **PX4 v1.17.0** flight stack, talking to ROS 2 over PX4's own uXRCE-DDS
  bridge — plain ROS 2 topics end to end, no MAVLink translation hop
- **ROS 2 Humble** throughout
- **Gazebo Harmonic** simulation — headless by default (no GPU required), with
  an opt-in real GUI window to watch flights visually
- **Modular, pluggable missions** — adding a new flight pattern means adding
  one file, not editing a monolithic script
- **One autonomy codebase, two targets**: sim (Gazebo, dev host) and real
  hardware (Orange Pi 5 Plus + Pixhawk 6) run identical mission/control code
- A clean extension point for **SLAM and Nav2** once basic flight and VIO are
  solid (not yet built — see [Roadmap](#roadmap))

---

## Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                     common_autonomy (ROS 2 workspace)                │
│           SHARED — identical source, sim and real hardware          │
│  common_control/  common_missions/  common_perception/ (VIO)         │
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
        │                   │            │  see Roadmap below  │
        │                   │            │                     │
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
   armed, when it's reached a waypoint, and when to land — see the
   [ROS 2 Topics Reference](resource/reference.md#sec-6) for the exact list.

For the full phase-by-phase design (why each version pin was chosen, what
each future phase covers), see `IMPLEMENTATION_PLAN.md` in this repo — it's a
local working document (gitignored, not pushed) since it changes fast during
active development.

---

## Quick Start

On any Linux box with Docker installed (see the
[Setup Guide](resource/setup-guide.md#sec-3) if you need to install Docker
itself):

```bash
git clone https://github.com/abhishekbera86/gps-denied-drone.git
cd gps-denied-drone

make build          # builds both Docker images — ~15-40 min the first time
make sim            # starts PX4 SITL + Gazebo (headless) + the ROS 2 bridge
make build-ws       # colcon-builds the ROS 2 workspace — one-time

make flight-test    # arm -> takeoff -> hover -> land -> disarm, in sim, GPS
```

Want to watch it fly instead of running headless? Use `make sim-gui` instead
of `make sim` — pops the real Gazebo window (any halfway-recent Intel iGPU is
enough, no dedicated GPU needed). Want to fly GPS-denied on VIO instead of
GPS?

```bash
PX4_GZ_WORLD=vio_test make sim-gui
LOCALIZATION=vision VIO_BACKEND=openvins make flight-test
```

That's the whole loop. For the complete walkthrough with every command
explained — including flying a full waypoint mission, inspecting the running
system, and shutting down cleanly — continue to the
**[Setup Guide](resource/setup-guide.md)** and then the
**[Mission Testing Guide](resource/mission-testing.md)**.

---

## Documentation

The essentials are on this page. Everything else — full walkthroughs,
exhaustive reference tables, and the complete bug/fix history — lives in
`resource/`, one focused doc per topic:

| Guide | What's in it |
|---|---|
| **[Setup Guide](resource/setup-guide.md)** | Prerequisites, cloning, building the Docker images, starting the simulation (headless or GUI, choosing a Gazebo world), building the ROS 2 workspace. Start here on a fresh machine. |
| **[Mission Testing Guide](resource/mission-testing.md)** | Waiting for PX4 to be ready, flying the hover test, flying a full waypoint mission, inspecting the running system live, stopping the stack cleanly. |
| **[Technical Reference](resource/reference.md)** | Every `make` command, every ROS 2 topic and parameter this stack actually uses, every launch file, every environment variable — plus the full GPS/VIO localization-switching mechanism. |
| **[Known Issues & Fixes](resource/known-issues.md)** | 37 real bugs, gotchas, and dead ends hit building and flying this stack, and exactly how (or whether) each was fixed. Read this before re-debugging something that's already been solved. |
| **[Localization Source Design](resource/phase3-gps-denied-localization-source.md)** | The full design rationale and debugging history behind GPS/VIO switching — the *why* behind the Technical Reference's *how*. |

---

## Repository Layout

```
docker/
  Dockerfile.px4_sim              PX4 v1.17.0 SITL + Gazebo Harmonic (headless default, GUI opt-in)
  Dockerfile.ros2_autonomy        ROS 2 Humble + uXRCE-DDS agent + px4_msgs/px4_ros_com + OpenVINS
  entrypoint_ros2_autonomy.sh     Auto-starts the Micro-XRCE-DDS-Agent at container boot
  px4_sitl_overrides/             SITL-only PX4 param overrides (Known Issues #4)
  px4_sitl_worlds/                Overlay Gazebo worlds (empty.sdf, vio_test.sdf — Setup Guide)
  px4_sitl_models/                Overlay Gazebo model: this repo's own D435i camera (see below)
docker-compose.yml                sim profile: px4-sim + ros2-autonomy (host networking)
docker-compose.gui.yml            Opt-in overlay: X11/DRI passthrough for `make sim-gui`
.env                              Version pins + runtime config (Technical Reference)
Makefile                          build / sim / sim-gui / flight-test / mission / gz-resync / stop / shell
resource/
  setup-guide.md                  Prerequisites + build/start walkthrough
  mission-testing.md              Flying and inspecting the stack
  reference.md                    Commands, topics, parameters, launch files, env vars
  known-issues.md                 37 real bugs and fixes hit during bring-up
  phase3-gps-denied-localization-source.md   Full VIO/localization-switch design + debugging history
ros2_ws/src/
  common_control/                 OffboardControlNode — heartbeat/arm/offboard/
                                   takeoff/waypoints/hover/land state machine, including
                                   the post-landing disarm fallback (Phase 1 ✓)
  common_missions/                MissionBase + the square mission + the shared,
                                   transport-agnostic autonomy.launch.py (Phase 2 ✓)
  common_perception/               Localization-source switch (GPS/vision) + both VIO
                                   backends (loopback stand-in, real OpenVINS) (Phase 3 ✓)
                                   + state_tf_publisher/viz.launch.py — RViz2 live TF/path
                                   view, sim/hw-agnostic
  sim_bringup/                    Sim-only launch + params (sim_params.yaml) — includes
                                   autonomy.launch.py, no flight logic of its own (Phase 2.5 ✓)
  hw_bringup/                     Real-hardware bringup STUB — serial uXRCE-DDS agent +
                                   hw_params.yaml; untested, no hardware yet (Phase 4 seam)
```

---

## No Upstream Repos Are Forked or Modified

Everything external — PX4-Autopilot, px4_msgs, px4_ros_com,
Micro-XRCE-DDS-Agent, OpenVINS — is cloned **read-only at pinned versions
from the official public repos during `docker build`**. All customization
lives as small overlay files inside *this* repo, `COPY`'d into the image
*after* the clone step, at build time — the same mechanism every time,
whether it's a PX4 param override, a world, or a whole Gazebo model:

| What | Overlay file(s) | Overwrites/adds |
|---|---|---|
| PX4 arming param override | `docker/px4_sitl_overrides/4002_gz_x500_depth.post` | A SITL-only airframe hook |
| Gazebo worlds | `docker/px4_sitl_worlds/empty.sdf`, `vio_test.sdf` | `Tools/simulation/gz/worlds/` |
| Gazebo camera model | `docker/px4_sitl_models/d435i/` (new model), `docker/px4_sitl_models/x500_d435i_depth/model.sdf` (swaps the stock `x500_depth`'s camera from a generic OakD-Lite to this repo's own D435i model) | `Tools/simulation/gz/models/` |

Nothing to fork, nothing to patch by hand, no upstream maintenance burden: to
replicate the system anywhere, this repo + Docker is the complete recipe.

Full details: [Technical Reference](resource/reference.md).

---

## Roadmap

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
  sim/real split (launch + params only, never flight logic — see
  [Architecture](#architecture)), plus an opt-in Gazebo GUI (`make sim-gui`)
  to watch flights visually. `hw_bringup` ships as an untested stub (serial
  DDS agent + param file) — real hardware wiring is still Phase 4. Every
  tuning value (takeoff height, mission geometry, …) is a required parameter
  with no code-side default — it must come from `sim_params.yaml`/
  `hw_params.yaml` ([Technical Reference](resource/reference.md#sec-7)) or
  startup fails loudly — and `make build-ws` is split from `make
  flight-test`/`make mission` so editing that YAML never triggers a rebuild.
- **Phase 3 — complete**: GPS-denied flight in sim
  ([Technical Reference](resource/reference.md#sec-8)). Milestone A
  (`VIO_BACKEND=loopback`, a zero-drift fake-VIO stand-in) proved the
  GPS/vision switch mechanism end to end. Milestone B (`VIO_BACKEND=openvins`,
  real monocular VIO) flies both missions GPS-denied, including landing and
  auto-disarm, confirmed on repeated clean retests after fixing several real
  bugs found via actual flight testing
  ([Known Issues 17-25](resource/known-issues.md#issue-17)) — camera
  extrinsics (wrong, then wrong direction), a missing EKF2 sensor lever arm,
  IMU-excitation starvation from too-slow cruise speeds, visual feature
  starvation near the ground, and a post-landing disarm gap. Treat as a
  strong, well-evidenced fix rather than an absolute guarantee — this
  project's own testing saw a config fly clean multiple times, then diverge,
  with nothing changed, before the real root causes were found. Full
  incident history:
  [resource/phase3-gps-denied-localization-source.md](resource/phase3-gps-denied-localization-source.md).
- **Phase 4**: same code on real hardware — Orange Pi 5 Plus + Pixhawk 6C +
  RealSense D435i. Not started; `hw_bringup` remains an untested stub.
- **Phase 5**: SLAM + Nav2 navigation.

Full phase-by-phase detail lives in `IMPLEMENTATION_PLAN.md` (local,
gitignored, not pushed — an internal working document that changes too fast
to keep in sync with the public repo).
