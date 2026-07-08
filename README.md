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

## 4. Quick Start — Replicating on a Fresh Machine

The full flow on any Linux box that has Docker (nothing else required):

```bash
# Step 1 — clone THIS repo (the only repo you ever clone or touch)
git clone https://github.com/abhishekbera86/gps-denied-drone.git
cd gps-denied-drone

# Step 2 — build the two images (first time only)
make build

# Step 3 — start the simulation stack
make sim

# Step 4 — wait ~30-60 s (PX4's EKF2 estimator must converge after boot),
#          then fly the takeoff-hover-land test in a second terminal
make flight-test

# Step 5 — fly a waypoint mission (square by default; MISSION=survey for the
#          lawnmower coverage pattern)
make mission
make mission MISSION=survey
```

### What each step does

`make build` compiles two images (expect ~15–40 min on the very first run,
depending on CPU and network — later rebuilds hit Docker layer cache and
take seconds):
- `px4-sim` — clones PX4-Autopilot at the pinned `v1.17.0` tag from the
  official repo, installs Gazebo Harmonic via PX4's own
  `Tools/setup/ubuntu.sh`, pre-compiles the `px4_sitl_default` target, and
  overlays this repo's SITL-only param file (see §5, issue 4).
- `ros2-autonomy` — ROS 2 Humble + the Micro-XRCE-DDS-Agent (built from
  source at `v3.0.1`) + `px4_msgs` (`release/1.17`) / `px4_ros_com`
  (`release/1.16`), colcon-built against PX4 v1.17.0 message definitions.

`make sim` starts both containers. The ros2-autonomy entrypoint auto-starts
the Micro-XRCE-DDS-Agent, so the PX4 ↔ ROS 2 bridge comes up on its own —
no manual step. Verify with `make logs`: you should see Gazebo Harmonic
spawn `x500_depth_0` and `uxrce_dds_client` connect to `127.0.0.1:8888`.

`make flight-test` colcon-builds the `common_control` package inside the
container (source is live-mounted from `ros2_ws/src/` — edit on host, no
image rebuild needed) and runs `offboard_control_node`, which:
1. Streams the `OffboardControlMode` heartbeat + `TrajectorySetpoint` at 10 Hz
2. Switches PX4 to offboard mode and arms (retrying once per second until
   PX4 confirms via `vehicle_status` — right after boot PX4 rejects arming
   until its preflight checks pass, so early retries are normal)
3. Takes off to 2 m, hovers 5 s, lands, and PX4 auto-disarms on touchdown

Expected output ends with:
```
[INFO] [...]: Armed and offboard — climbing to takeoff height
[INFO] [...]: Reached takeoff height — hovering for 5.0s
[INFO] [...]: Hover complete — landing
[INFO] [...]: Landed and disarmed — mission complete
```

`make mission` runs a named waypoint mission from the `common_missions`
package via `ros2 launch common_missions mission.launch.py mission:=<name>`:
- `square` (default) — 4 m square at takeoff height, nose pointed along each
  leg, landing back at the start.
- `survey` — lawnmower coverage of an 8 m × 6 m rectangle with 2 m lane
  spacing, returning to the start to land.

Both are subclasses of `MissionBase`, which reuses the entire Phase 1
arm/offboard/takeoff/land state machine and only supplies waypoints — a new
mission is ~25 lines (declare geometry parameters, return a waypoint list).
Waypoint arrival is judged by position tolerance (0.5 m default), never by
elapsed time, so missions behave identically on slow and fast hosts.

### Inspecting the running system

```bash
make shell            # bash inside ros2-autonomy (ROS 2 side)
# then: ros2 topic list / ros2 topic hz /fmu/out/vehicle_odometry (~10-15 Hz)

make shell-px4        # bash inside px4-sim (flight-controller side)
# PX4 internals are queryable via the px4-* client binaries, e.g.:
# cd /PX4-Autopilot/build/px4_sitl_default/rootfs && \
#   /PX4-Autopilot/build/px4_sitl_default/bin/px4-listener vehicle_status
```

### Stopping / restarting

```bash
make stop
```

**Always restart the whole stack together** (`make stop && make sim`).
Restarting only the px4-sim container leaves the DDS bridge wedged — see §5,
issue 5.

### No upstream repos are forked or modified

Everything external — PX4-Autopilot, px4_msgs, px4_ros_com,
Micro-XRCE-DDS-Agent — is cloned **read-only at pinned versions from the
official public repos during `docker build`**. All customization lives as
small overlay files inside *this* repo (e.g.
`docker/px4_sitl_overrides/4002_gz_x500_depth.post`, copied into the image
at build time). Nothing to fork, nothing to patch by hand, no upstream
maintenance burden: to replicate the system anywhere, this repo + Docker is
the complete recipe.

---

## 5. Known Issues Hit During Bring-Up (already fixed in this repo)

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
time simply advances slower than real time — on this project's dev laptop
the 2 m climb takes ~70 s of wall time. Nothing is hung; the flight is
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

---

## 6. Repository Layout

```
docker/
  Dockerfile.px4_sim              PX4 v1.17.0 SITL + Gazebo Harmonic (headless)
  Dockerfile.ros2_autonomy        ROS 2 Humble + uXRCE-DDS agent + px4_msgs/px4_ros_com
  entrypoint_ros2_autonomy.sh     Auto-starts the Micro-XRCE-DDS-Agent at container boot
  px4_sitl_overrides/             SITL-only PX4 param overrides (see §5, issue 4)
docker-compose.yml                sim profile: px4-sim + ros2-autonomy (host networking)
.env                              Version pins (PX4, px4_msgs/px4_ros_com, Gazebo model)
Makefile                          build / sim / flight-test / mission / stop / shell / logs
ros2_ws/src/
  common_control/                 OffboardControlNode — heartbeat/arm/offboard/
                                   takeoff/waypoints/hover/land state machine (Phase 1 ✓)
  common_missions/                MissionBase + square/survey missions, selected
                                   via mission:= launch arg (Phase 2 ✓)
  (common_perception, sim_bringup, hw_bringup — future phases)
```

---

## 7. Command Reference

| Command | Action |
|---|---|
| `make build` | Build `px4-sim` + `ros2-autonomy` images |
| `make sim` | Start PX4 SITL (Gazebo Harmonic, headless) + the ROS 2 bridge container |
| `make flight-test` | Build `common_control` in-container and fly takeoff → hover → land |
| `make mission` | Fly a waypoint mission: `MISSION=square` (default) or `MISSION=survey` |
| `make shell` | Bash inside `ros2-autonomy` |
| `make shell-px4` | Bash inside `px4-sim` |
| `make logs` | Tail logs from both containers |
| `make ps` | Show container status |
| `make stop` | Stop and remove both containers |
| `make clean` | Remove locally built images |
| `make clean-all` | Remove images **and** the colcon build cache volume |

---

## 8. Roadmap

This repo is under active development.

- **Phase 0 — complete**: PX4 SITL + Gazebo Harmonic boots headless; the
  uXRCE-DDS bridge exposes the full `/fmu/*` topic set to ROS 2.
- **Phase 1 — complete**: `common_control/offboard_control_node` flies the
  full cycle in sim — arm → offboard → takeoff → hover → land → disarm
  (`make flight-test`).
- **Phase 2 — complete**: pluggable missions (`common_missions`: square,
  survey, selected by name via `make mission MISSION=<name>`) on top of the
  Phase 1 control primitives.
- **Phase 3**: GPS-denied flight in sim — OpenVINS VIO fed by the simulated
  depth camera, fused into PX4 EKF2 with GPS disabled.
- **Phase 4**: same code on real hardware — Orange Pi 5 Plus + Pixhawk 6 +
  RealSense D435i.
- **Phase 5**: SLAM + Nav2 navigation.

Full phase-by-phase detail lives in `IMPLEMENTATION_PLAN.md` (local,
gitignored, not pushed — an internal working document that changes too fast
to keep in sync with the public repo).
