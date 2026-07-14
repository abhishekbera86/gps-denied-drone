# Technical Reference

> Part of the [GPS-Denied Autonomous Drone Stack](../README.md) documentation set.

The exhaustive reference: every `make` command, every ROS 2 topic and
parameter this stack actually uses, how GPS/VIO localization switching
works, every launch file, and every environment variable. For a guided
walkthrough instead, see the [Setup Guide](setup-guide.md) and
[Mission Testing Guide](mission-testing.md).

## Contents

- [Command Reference](#sec-5)
- [ROS 2 Topics Reference](#sec-6)
- [ROS 2 Parameters Reference](#sec-7)
- [Localization Source: GPS or VIO](#sec-8)
- [Launch Files Reference](#sec-9)
- [Environment Variables Reference](#sec-10)

---

<a id="sec-5"></a>

## 5. Command Reference

Every `make` target, what it does, and any parameters it accepts. Parameters
are passed as `VAR=value` after the target name (standard `make` override
syntax) or as an environment variable before it.

| Command | Parameters | What it does |
|---|---|---|
| `make build` | — | Builds the `px4-sim` and `ros2-autonomy` **Docker images**. One-time, or after editing a Dockerfile. |
| `make build-ws` | — | Colcon-builds the **ROS 2 workspace** inside `ros2-autonomy`. One-time, or after adding/removing a package or editing `setup.py`/`package.xml` — NOT needed after editing Python/launch/config files ([§4.4](setup-guide.md#sec-4-4)). |
| `make sim` | `PX4_GZ_WORLD=empty` *(default)* or `vio_test` | Starts PX4 SITL (Gazebo Harmonic, **headless**) + the ROS 2 bridge container. |
| `make sim-gui` | `PX4_GZ_WORLD=` (same as above) · `GZ_SW_RENDER=1` *(optional)* — forces software (llvmpipe) rendering if the GPU path fails | Same stack as `make sim`, but with the real Gazebo GUI window forwarded to the host X display, plus `rqt-viewer` (live `/camera/camera/color/image_raw` preview) and `rviz2` (live TF + flown path only, [§4.3](setup-guide.md#sec-4-3)) — both up from the start so neither can lose the sequencing race against a mission's camera bridge. `rviz2` needs `make build-ws` to have run at least once. **Always pass `PX4_GZ_WORLD` on the command line, never edit it into `.env`** — see [issue 19](known-issues.md#issue-19). |
| `make flight-test` | `LOCALIZATION=gps` *(default)* or `vision` · `VIO_BACKEND=loopback` *(default)* or `openvins` (only when `LOCALIZATION=vision`, and only with `PX4_GZ_WORLD=vio_test` — [§8](reference.md#sec-8)) | Flies the hover test (arm → takeoff → hover → land) via `sim_bringup`. Requires `make build-ws` first. |
| `make mission` | `MISSION=square` *(default, and currently the only mission)* · `LOCALIZATION=` / `VIO_BACKEND=` (same as above) | Flies the named waypoint mission via `sim_bringup`. Requires `make build-ws` first. |
| `make gz-resync` | — | Fallback: forces the Gazebo GUI to rebroadcast its full scene, for the rare case the drone doesn't appear in the GUI's 3D view ([§4.3](setup-guide.md#sec-4-3), [issue 20](known-issues.md#issue-20)). No effect on headless mode. |
| `make shell` | — | Opens a bash shell inside `ros2-autonomy` (the ROS 2 / DDS-bridge container). |
| `make shell-px4` | — | Opens a bash shell inside `px4-sim` (the PX4 flight-controller container). |
| `make logs` | — | Tails logs from both containers (`Ctrl-C` to stop tailing; containers keep running). |
| `make ps` | — | Shows container status (`docker compose ps`). |
| `make stop` | — | Stops and removes both containers. Data in the colcon build cache volume survives. |
| `make clean` | — | Removes the locally built `px4-sim`/`ros2-autonomy` images (keeps the colcon cache volume). |
| `make clean-all` | — | Removes images **and** the colcon build cache volume — the next build/run starts completely fresh. |

`.env` also drives several build/runtime values (Gazebo model, world, version
pins) without needing a `make` parameter at all — see
[§9](reference.md#sec-9).

---

<a id="sec-6"></a>

## 6. ROS 2 Topics Reference

Every ROS 2 topic that `common_control`/`common_missions` actually publishes
or subscribes to, over the PX4 uXRCE-DDS bridge.

| Topic | Direction | Message Type | Rate | Purpose |
|---|---|---|---|---|
| `/fmu/in/offboard_control_mode` | Publish | `px4_msgs/msg/OffboardControlMode` | 10 Hz (`control_rate_hz`) | Heartbeat telling PX4 "position control, offboard, still here" — required continuously or PX4 drops offboard mode. |
| `/fmu/in/trajectory_setpoint` | Publish | `px4_msgs/msg/TrajectorySetpoint` | 10 Hz | The current target `(x, y, z, yaw)` in NED, streamed even while disarmed ([issue 7](known-issues.md#issue-7)). |
| `/fmu/in/vehicle_command` | Publish | `px4_msgs/msg/VehicleCommand` | On demand | Arm/disarm, offboard mode switch, and `NAV_LAND` commands. |
| `/fmu/out/vehicle_local_position_v1` | Subscribe | `px4_msgs/msg/VehicleLocalPosition` | ~as published by PX4 | Feeds the waypoint-arrival check (`waypoint_tolerance_m` distance test) and the takeoff-height check. |
| `/fmu/out/vehicle_status_v1` | Subscribe | `px4_msgs/msg/VehicleStatus` | ~as published by PX4 | `arming_state` and `nav_state` — drives every state-machine transition (armed? in offboard? disarmed after landing?). |
| `/fmu/out/vehicle_land_detected` | Subscribe | `px4_msgs/msg/VehicleLandDetected` | ~as published by PX4 | Only `has_low_throttle` is read (a pure actuator-thrust signal) — drives the post-landing disarm fallback ([§7](reference.md#sec-7), [§8](reference.md#sec-8)) when PX4's own `arming_state` doesn't flip on its own. |

Notes:
- The `_v1` suffix is not a typo — PX4 v1.17 publishes **versioned** topic
  names for any message carrying a `MESSAGE_VERSION` field. The unversioned
  names used in older PX4 examples receive nothing on this version ([issue 6](known-issues.md#issue-6)).
- QoS on every topic above is `BEST_EFFORT` reliability + `TRANSIENT_LOCAL`
  durability + `KEEP_LAST` depth 1 (matching PX4's own DDS QoS) — see
  `PX4_QOS` in `common_control/common_control/offboard_control_node.py` if
  you're writing a new node against this bridge.

### `common_perception` topics ([§8](reference.md#sec-8) — localization source)

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

**`rviz2` live-viz topics ([§4.3](setup-guide.md#sec-4-3))** — published by `state_tf_publisher`
(`common_perception`), started by `make sim-gui`'s `rviz2` container via
`viz.launch.py`. Sim/hw-agnostic: sourced only from
`/fmu/out/vehicle_odometry`, so identical on real hardware.

| Topic | Direction | Message Type | Rate | Purpose |
|---|---|---|---|---|
| `tf` (`odom` → `base_link`) | Publish | `tf2_msgs/msg/TFMessage` | 10 Hz | The vehicle's live pose, converted from PX4's NED/FRD to ROS ENU/FLU via `frame_transforms` (reused unchanged from the VIO bridge — both conversions are their own inverse). |
| `/drone/path` | Publish | `nav_msgs/msg/Path` | 10 Hz | The path actually flown, `odom`-frame — a rolling buffer capped at 20000 poses (~30+ min of history) so a long-running viz session can't grow it unbounded. |

---

<a id="sec-7"></a>

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
([§4.4](setup-guide.md#sec-4-4)).

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
| `max_velocity_m_s` | float | `1.0` (hover) / `0.8` (missions) | `0.5` (hover) / `0.1` (missions) | Cruise speed cap toward any setpoint (takeoff, hover position, or a waypoint), in m/s. Without this, PX4's own offboard position controller accelerates toward every setpoint at its internal velocity limit — much faster than useful for watching a mission or a first real flight. Implemented as a feed-forward velocity vector aimed at the target, magnitude-capped at this value (`_capped_velocity_toward` in `offboard_control_node.py`) — not a PX4 firmware parameter. **Don't set this below ~0.5 m/s for `VIO_BACKEND=openvins` flights** — a real, confirmed failure mode: too slow starves monocular VIO of the acceleration events it needs to keep its scale/bias estimate converged, and a long, near-constant-velocity flight lets that error build up for the whole mission ([§8](reference.md#sec-8), [issue 17](known-issues.md#issue-17)). |
| `land_disarm_low_throttle_dwell_s` | float | `3.0` | `4.0` | Post-landing disarm fallback ([§8](reference.md#sec-8), [issue 21](known-issues.md#issue-21)): how long PX4's own actuator thrust must stay continuously low before this node explicitly force-disarms, used only if PX4's own auto-disarm doesn't happen on its own. Any single higher-throttle reading resets this to zero. |
| `land_disarm_max_timeout_s` | float | `60.0` | `90.0` | Ceiling on how long to wait for the condition above — if exceeded, this node gives up rather than ever disarming based on elapsed time alone, and logs an error requiring manual `px4-commander disarm -f`. Never the trigger by itself, only a bound on the wait. |
| `geofence_margin_m` | float | `2.0` | `1.0` | Geofence ([issue 24](known-issues.md#issue-24)): horizontal margin added on every side of the current route's bounding box (origin + every queued waypoint — auto-derived, not a separately hand-maintained box; see `_geofence_bounds` in `offboard_control_node.py`). A position outside this box in ANY flying state, including `LAND` ([issue 33](known-issues.md#issue-33)), aborts. |
| `geofence_height_margin_m` | float | `1.5` | `1.0` | Geofence altitude cap: how far above the highest point the route actually visits (usually `takeoff_height_m`) the vehicle may climb before the same abort triggers. |
| `geofence_hard_limit_m` | float | `3.75` | `10.0` (untested placeholder) | Geofence ([issue 33](known-issues.md#issue-33)): an absolute x/y clamp on the bound above, independent of mission geometry — whichever of (bbox+margin) or (hard limit) is smaller wins. Sim's `3.75` is derived from `vio_test.sdf`'s actual fence position (±4.75m), guaranteeing ≥1m wall clearance regardless of any mission's own margin math. hw's `10.0` is a placeholder, NOT derived from a real test area — must be retuned before free flight. |
| `estimate_invalid_abort_dwell_s` | float | `1.5` | `2.0` | Estimate-health watchdog ([issue 34](known-issues.md#issue-34)): how long PX4's own `xy_valid`/`v_xy_valid` must stay continuously false before aborting — source-agnostic (GPS or vision), see `_check_estimate_health` in `offboard_control_node.py`. Checked in every flying state including `LAND`, same during-land force-disarm shape as the geofence. |

### `square_mission`

| Parameter | Type | `sim_params.yaml` | `hw_params.yaml` | Meaning |
|---|---|---|---|---|
| `side_length_m` | float | `3.0` | `2.0` | Side length of the square flight path, in meters. The square is centered on the takeoff point (corners at ±half this value), not flown out of one corner — see [§4.7](mission-testing.md#sec-4-7). |

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

<a id="sec-8"></a>

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

**`VIO_BACKEND=openvins` needs `PX4_GZ_WORLD=vio_test`** ([§4.3](setup-guide.md#sec-4-3)) — start the
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

| `LOCALIZATION=` | `EKF2_GPS_CTRL` | `EKF2_EV_CTRL` | `EKF2_EV_POS_X/Y/Z` | `EKF2_EVP_NOISE` / `EKF2_EVV_NOISE` (floor) | Height reference |
|---|---|---|---|---|---|
| `gps` (default) | `7` (HPOS+VPOS+VEL) | `0` (off) | untouched | untouched | GPS |
| `vision` | `0` (off) | `5` (HPOS+VEL) | `0.12 / -0.03 / -0.242` | `0.3 m` / `0.15 m/s` | Baro (automatic fallback) |

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
contributor to a landing-divergence bug this project hit ([issue 18](known-issues.md#issue-18)).

`EKF2_EVP_NOISE`/`EKF2_EVV_NOISE` (`vision` only, added 2026-07-10) fix a
different real bug: PX4's default `EKF2_EV_NOISE_MD=0` takes vision
measurement noise from the message itself, using these two parameters only
as a LOWER BOUND — and `openvins_odometry_bridge` forwards OpenVINS's own
raw per-frame covariance untouched, so EKF2's trust in vision can swing up
toward however confident OpenVINS's estimate feels on a given frame, past
PX4's own `0.1 m`/`0.1 m/s` default floor — a real, confirmed contributor
to flight-to-flight instability (accurate on one `square` mission run,
drifting/hitting the fence on the next, nothing else changed) — see [issue 28](known-issues.md#issue-28) and `resource/Vio_Drift_analysis.txt`. Raised to `0.3 m`/`0.15
m/s` here — starting values, not derived from first principles, retune
from live flight results. **`EKF2_EV_NOISE_MD` is deliberately left
untouched at PX4's default (`0`)** — setting it to `1` ("ignore the
message's covariance entirely, use only the fixed params") was tried
first and reverted: confirmed live to break arming outright (PX4 stuck at
`arming_state=STANDBY`, "Preflight Fail: ekf2 missing data") across two
clean, fully-restarted attempts, isolated via a third identical attempt
that only reverted this one param and armed/flew/landed normally. See [issue 28](known-issues.md#issue-28) for the full isolation.

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
  Gazebo model, `docker/px4_sitl_models/d435i` — [§11](../README.md#no-upstream-repos-are-forked-or-modified) — not PX4's stock
  camera) to the exact ROS 2 topic names/types the real RealSense D435i's
  `realsense-ros` driver produces (`/camera/camera/color/image_raw`,
  `/camera/camera/imu` — so sim and real hardware present as literally "the
  same camera" to everything downstream, not just matched topic names),
  OpenVINS (`ov_msckf`) runs mono VIO against that feed, and
  `openvins_odometry_bridge` converts its output (ENU/FLU → NED/FRD) onto
  the same target topic. Requires `make build` ([§4.2](setup-guide.md#sec-4-2)) to have built OpenVINS
  — see below — and `PX4_GZ_WORLD=vio_test` at sim-start time ([§4.3](setup-guide.md#sec-4-3)).

### Building the VIO pipeline (`VIO_BACKEND=openvins`)

OpenVINS and a Gazebo-Harmonic-matched `ros_gz_bridge` are baked into the
`ros2-autonomy` image (`docker/Dockerfile.ros2_autonomy`) — `make build`
([§4.2](setup-guide.md#sec-4-2)) builds them along with everything else; there is no separate install
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
assuming a design was correct ([§12](known-issues.md), [issues 17-25](known-issues.md#issue-17)) — is in
`resource/phase3-gps-denied-localization-source.md`.

`common_control`/`common_missions` are completely unaware of any of this —
they only ever read PX4's already-fused `vehicle_local_position_v1`.

---

<a id="sec-9"></a>

## 9. Launch Files Reference

| File | Package | Arguments | Example |
|---|---|---|---|
| `sim.launch.py` | `sim_bringup` | `action` (`hover`\|`mission`, default `mission`) · `mission` (`square`, default `square` — currently the only mission) · `localization_source` (`gps`\|`vision`, default `gps`) · `vio_backend` (`loopback`\|`openvins`, default `loopback`, only used when `localization_source:=vision`) · `mavlink_url` (default `udpin:0.0.0.0:14540`) | `ros2 launch sim_bringup sim.launch.py action:=mission mission:=square localization_source:=vision vio_backend:=openvins` |
| `hw.launch.py` *(Phase 4 stub, untested)* | `hw_bringup` | `serial_device` (default `/dev/ttyUSB0`) · `baud` (default `921600`) · `action` · `mission` · `localization_source` · `vio_backend` · `mavlink_url` (UNTESTED SITL-shaped default) | `ros2 launch hw_bringup hw.launch.py serial_device:=/dev/ttyACM0 action:=mission mission:=square` |
| `autonomy.launch.py` | `common_missions` | `action` (`hover`\|`mission`) · `mission` (`square`) · `params_file` (path, optional) | `ros2 launch common_missions autonomy.launch.py action:=hover` |
| `mission.launch.py` | `common_missions` | `mission` (`square`, default `square`) | `ros2 launch common_missions mission.launch.py mission:=square` |
| `viz.launch.py` | `common_perception` | — | `ros2 launch common_perception viz.launch.py` |

`viz.launch.py` starts `state_tf_publisher` + RViz2 (`config/quad.rviz`) —
what `make sim-gui`'s `rviz2` container runs ([§4.3](setup-guide.md#sec-4-3), [§6](reference.md#sec-6)). Not included by
`sim.launch.py`/`hw.launch.py` on purpose: it's independent of any
mission's lifecycle, run separately so it survives across multiple
mission runs.

`sim.launch.py` and `hw.launch.py` are what `make mission`/`make
flight-test` actually run ([§5](reference.md#sec-5)) — they include `autonomy.launch.py` and pass
their own `params_file`. That's the entire sim-vs-real difference at the
launch layer: same `autonomy.launch.py`, different params file and DDS
transport. `mission.launch.py` is the standalone, transport-agnostic mission
selector `autonomy.launch.py` itself builds on; call it directly if you want
a mission with zero params-file involvement.

---

<a id="sec-10"></a>

## 10. Environment Variables Reference

Set in `.env` at the repo root, passed through by `docker-compose.yml` as
build args and/or runtime environment. Edit `.env` to change any of these —
no `make` parameter needed. **Never add an inline `# comment` after a
value** — see [issue 12](known-issues.md#issue-12).

| Variable | Default | Purpose |
|---|---|---|
| `PX4_VERSION` | `v1.17.0` | PX4 firmware tag cloned into `px4-sim`. Must match `PX4_MSGS_BRANCH`/`PX4_ROS_COM_BRANCH`. Flash this same version to the real Pixhawk 6 in Phase 4. |
| `PX4_MSGS_BRANCH` | `release/1.17` | `px4_msgs` branch built into `ros2-autonomy`. |
| `PX4_ROS_COM_BRANCH` | `release/1.16` | `px4_ros_com` branch built into `ros2-autonomy`. |
| `ROS_DISTRO` | `humble` | ROS 2 distribution. |
| `PX4_GZ_MODEL` | `x500_depth` | Gazebo vehicle model (x500 quad + simulated D435i — `docker/px4_sitl_models/d435i`, [§11](../README.md#no-upstream-repos-are-forked-or-modified)). |
| `PX4_GZ_WORLD` | *(deliberately unset — see below)* | Gazebo world by name: `empty` (default, bare ground) or `vio_test` (required for `VIO_BACKEND=openvins`, [§4.3](setup-guide.md#sec-4-3)/[§8](reference.md#sec-8)); any world PX4 bundles also works (`default`, `walls`, `baylands`, …) — including for the camera/IMU bridge, which templates its Gazebo topic paths from this SAME variable rather than hardcoding one world ([issue 36](known-issues.md#issue-36)), so a new VIO-capable world needs no YAML edits, just this value. **Set this on the command line only** (`PX4_GZ_WORLD=vio_test make sim-gui`) — **do not add a `PX4_GZ_WORLD=...` line to `.env`**, it will silently defeat the command-line override ([issue 19](known-issues.md#issue-19)), which is exactly why this row has no default value here. |
| `PX4_HEADLESS` | `1` | `1` = headless server-only Gazebo. Don't hand-edit this for GUI mode — use `make sim-gui`, which sets it via a compose overlay. |
| `UXRCE_DDS_PORT` | `8888` | Micro-XRCE-DDS-Agent UDP port (PX4 ↔ ROS 2 bridge). |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | ROS 2 middleware implementation. |
| `DRONE_NAMESPACE` | `drone1` | Reserved for future multi-vehicle namespacing; not yet consumed by any node. |
| `LIBREALSENSE_VERSION` | `2.58.2` | librealsense SDK version — must match the physical D435i firmware (Phase 4). |
| `PIXHAWK_SERIAL_PORT` | `/dev/ttyUSB0` | Serial device for the real Pixhawk 6 (Phase 4; matches `hw_bringup`'s `serial_device` default). |
| `PIXHAWK_BAUD_RATE` | `921600` | Serial baud rate for the real Pixhawk 6 (Phase 4). |


---

[← Back to README](../README.md) · [Known Issues & Fixes](known-issues.md)
