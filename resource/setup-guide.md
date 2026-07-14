# Setup Guide

> Part of the [GPS-Denied Autonomous Drone Stack](../README.md) documentation set.

Everything needed to go from a fresh Linux box to a running simulation stack:
prerequisites, cloning, building the Docker images, starting the simulation
(headless or GUI), and building the ROS 2 workspace. Once the stack is up,
continue to the [Mission Testing Guide](mission-testing.md) to actually fly.

## Contents

- [Prerequisites](#sec-3)
- [Step-by-Step Setup](#sec-4)
  - [4.1 Clone the Repo](#sec-4-1)
  - [4.2 Build the Docker Images](#sec-4-2)
  - [4.3 Start the Simulation](#sec-4-3)
  - [4.4 Build the ROS 2 Workspace](#sec-4-4)

---

<a id="sec-3"></a>

## 3. Prerequisites

### Host machine
- **OS**: Ubuntu 22.04 LTS or any Linux distribution running Docker
- **Architecture**: x86_64 for development/simulation; ARM64 (Orange Pi 5
  Plus) for real flight hardware
- **Graphics**: none required for headless mode (`make sim`) — Gazebo
  Harmonic runs fully headless by default. For GUI mode (`make sim-gui`, [§4.3](setup-guide.md#sec-4-3))
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

<a id="sec-4"></a>

## 4. Step-by-Step Setup and Usage Guide

This walks through the entire flow on any Linux box that has Docker, from a
fresh clone to a flown mission, one command at a time. Every command's exact
effect is explained inline; full reference tables (all commands, all ROS
topics, all parameters, all launch files) follow in [§5](reference.md#sec-5)–[§9](reference.md#sec-9).

<a id="sec-4-1"></a>

### 4.1 Clone the Repo

```bash
git clone https://github.com/abhishekbera86/gps-denied-drone.git
cd gps-denied-drone
```

This is the *only* repo you ever clone or touch. PX4-Autopilot, px4_msgs,
px4_ros_com, and the Micro-XRCE-DDS-Agent are all cloned automatically,
read-only, at pinned versions, inside the Docker build — see
[§11](../README.md#no-upstream-repos-are-forked-or-modified).

<a id="sec-4-2"></a>

### 4.2 Build the Docker Images

```bash
make build
```

Compiles two images (expect ~15–40 min on the very first run, depending on
CPU and network — later rebuilds hit Docker layer cache and take seconds):

- **`px4-sim`** — clones PX4-Autopilot at the pinned `v1.17.0` tag, installs
  Gazebo Harmonic via PX4's own `Tools/setup/ubuntu.sh`, pre-compiles the
  `px4_sitl_default` target, and overlays this repo's SITL-only param file
  ([issue 4](known-issues.md#issue-4)) and empty world ([issue 13](known-issues.md#issue-13)).
- **`ros2-autonomy`** — ROS 2 Humble + the Micro-XRCE-DDS-Agent (built from
  source at `v3.0.1`) + `px4_msgs` (`release/1.17`) / `px4_ros_com`
  (`release/1.16`), colcon-built against PX4 v1.17.0 message definitions.

Rebuild an individual image (e.g. after editing a Dockerfile) with
`docker compose --profile sim build <service>`, e.g.
`DOCKER_BUILDKIT=1 docker compose --profile sim build px4-sim`.

<a id="sec-4-3"></a>

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
mission's own process-exit path ([issue 25](known-issues.md#issue-25)) tears its camera bridge
down the instant the mission finishes, so a late viewer might never see a
single frame. The window stays blank/black until a vision mission is
actually flying; that's expected, not a bug — the GPS localization path
never starts a camera bridge at all. `make stop` tears it down along with
everything else.

`make sim-gui` also starts a fourth container, `rviz2` — an RViz2 window
showing the vehicle's live TF (`odom` → `base_link`) and the path it has
actually flown (`/drone/path`). **Deliberately TF/path only, no camera
image** — an earlier version also showed the camera feed in RViz, but that
meant RViz's `Image` display and `rqt-viewer` both subscribing to the same
raw ~83 MB/s stream (1280×720@30Hz), and RViz's `Image` plugin uploads a
fresh GPU texture on the same render thread as the 3D view — confirmed
live to visibly lag/go stale in a way `rqt-viewer`'s dedicated 2D widget
doesn't ([issue 28](known-issues.md#issue-28), `resource/Vio_Drift_analysis.txt`). Use
`rqt-viewer` for the camera; `rviz2` for spatial data. Unlike `rqt-viewer`
(a baked-in apt package), `rviz2`'s window runs this repo's own
`common_perception` package (`state_tf_publisher` node + `viz.launch.py`,
bind-mounted and colcon-built like every other node here) — **`make
build-ws` must have run at least once before `make sim-gui`**, or the
container exits immediately with a "Workspace not built yet" message
(check `make ps` / `make logs`). `state_tf_publisher` subscribes to
`/fmu/out/vehicle_odometry`, which PX4 publishes as soon as its estimator
initializes — no mission needed — so the TF/path appear immediately and
keep updating across multiple mission runs (this node is deliberately
*not* part of a mission's own launch/shutdown lifecycle, for the same "up
from the start" reason as `rqt-viewer` above). Sim/hw-agnostic by design
— `viz.launch.py` only touches topics PX4 and `common_perception` publish
identically on real hardware, so the same launch file is the intended
Phase 4 hardware-GUI viewer too,
unchanged.

**Choosing a world** — two exist, pick with `PX4_GZ_WORLD=`:

```bash
PX4_GZ_WORLD=empty make sim-gui       # default — bare ground+sun+grid, open space
PX4_GZ_WORLD=vio_test make sim-gui    # required for VIO_BACKEND=openvins ([§8](reference.md#sec-8))
```

`empty` (`docker/px4_sitl_worlds/empty.sdf`) is open space with nothing to
watch takeoff/missions through. `vio_test`
(`docker/px4_sitl_worlds/vio_test.sdf`) adds a ring of colored boxes plus a
ground-level tile grid, both **centered on the spawn point** (the vehicle
spawns at the world origin, which is also where PX4's local/odom frame
initializes and where every mission takes off and lands — fence at
±4.75 m, tiles ±4 m, so landing drift has maximum clearance on every
side; see [§4.7](mission-testing.md#sec-4-7)). The props are **required, not cosmetic**, if you're
flying `VIO_BACKEND=openvins`: monocular VIO needs real corner features
to track, and the plain `empty` world's flat gray plane gives it zero
(OpenVINS's initializer fails every frame). Both are repo-owned overlay
files, not a
fork of PX4 ([§11](../README.md#no-upstream-repos-are-forked-or-modified)) — same mechanism as the SITL param override, [issue 4](known-issues.md#issue-4).

**Setting `PX4_GZ_WORLD` in `.env` instead of on the command line does
not work and will not error** — the Makefile's own default is meant to be
overridden per-invocation (`PX4_GZ_WORLD=vio_test make sim-gui`), and a
value hardcoded in `.env` silently wins over that override every time (a
GNU Make quirk — confirmed as a real, hit bug, see [issue 19](known-issues.md#issue-19)). Always
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
the complete scene on connect, drone included. See [issue 20](known-issues.md#issue-20) for the
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

<a id="sec-4-4"></a>

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
([§7](reference.md#sec-7)) takes effect on the *next* `make flight-test`/`make mission` with no
rebuild at all — `make mission` and `make flight-test` deliberately don't
colcon-build for you every time, so tuning-and-reflying stays fast.

If you skip this step, `make flight-test`/`make mission` will tell you to
run it — they check for the built workspace rather than silently failing.


---

**Next:** [Mission Testing Guide](mission-testing.md) — fly the hover test and your first mission.

[← Back to README](../README.md)
