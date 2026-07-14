# Development Status Tracker

> Local-only file (gitignored, not pushed). Purpose: let anyone — including a
> fresh Claude Code session — pick up exactly where this left off without
> re-deriving context. Update this as work progresses. See
> `IMPLEMENTATION_PLAN.md` for the full phase-by-phase design; this file is
> just "where are we right now."

Last updated: 2026-07-09

---

## Current phase: Phase 3 Milestone B (real OpenVINS) — part 6's in-flight
## divergence hazard has been ROOT-CAUSED AND FIXED (part 7), confirmed on 2
## consecutive clean, fully-automatic mission retests (zero manual
## intervention, ground-truth-verified safe landings both times). Three
## contributing causes, each fixed: (1) the 0.2 m/s cruise speed starved
## OpenVINS of IMU excitation for its whole ~90s flight — accelerometer
## bias never converged before landing (confirmed via OpenVINS's own debug
## log); raised to 0.8 m/s. (2) `EKF2_EV_POS_X/Y/Z` (the vision sensor's
## lever arm from the flight controller's IMU) was never configured
## anywhere in this repo — now set automatically on vision-source switch.
## (3) the vio_test world's ground plane was flat gray with only tall
## props for texture, starving the tracker of features at low altitude —
## added a 64-tile ground-level checkerboard. `has_low_throttle` fallback
## disarm (part 6) still in place as a safety backstop regardless. Treat as
## a strong, well-evidenced fix (see part 7's honest caveat on remaining
## run-to-run variance risk), not an absolute guarantee — this project has
## already seen a config work clean 3x then diverge 2x in the same session.
## Two earlier extrinsics bugs (wrong rotation, then wrong direction —
## parts 2/3) remain fixed. Target real hardware: Pixhawk 6C + Intel
## RealSense D435i (VIO) + Orange Pi 5 Plus/Ubuntu 22.04 (companion
## computer). See part 7 below for the full analysis and fix.

## Done
- Full redesign from Aerostack2 (v2) to PX4-native ROS 2 over uXRCE-DDS (v3).
  See `IMPLEMENTATION_PLAN.md` for why.
- Removed all AS2-era files: `quad_core`/`quad_sim`/`quad_real` packages,
  `Dockerfile.as2`/`.hw`/`.px4_sitl`/`.ros2`/`.vio`, `config/openvins`,
  `config/px4_params`, old scripts (`launch_sim.sh`, `health_check.sh`, etc.)
- `docker/Dockerfile.px4_sim` — builds and boots. PX4 v1.17.0 SITL + Gazebo
  Harmonic 8.14.0, headless, `x500_depth` model.
- `docker/Dockerfile.ros2_autonomy` — builds. ROS 2 Humble + Micro-XRCE-DDS-Agent
  v3.0.1 + `px4_msgs` (release/1.17) + `px4_ros_com` (release/1.16).
- `docker-compose.yml`, `.env`, `Makefile` rewritten for the 2-container
  `sim` profile (`px4-sim`, `ros2-autonomy`). No `hw`/`vio` profiles yet —
  intentionally not added until Phase 3/4 build them for real.
- **Phase 0 smoke test passed**: `ros2 topic list` inside `ros2-autonomy`
  shows the full PX4 topic set (`/fmu/in/*`, `/fmu/out/*`) over the DDS
  bridge; `/fmu/out/vehicle_odometry` confirmed streaming at ~13 Hz.

## Phase 1 result (2026-07-07)
- `ros2_ws/src/common_control` exists and WORKS: `offboard_control_node`
  completed the full cycle against SITL — arm → offboard → takeoff to 2 m →
  hover 4 s → AUTO_LAND → touchdown → auto-disarm. Verified via
  `px4-listener vehicle_status` / `vehicle_land_detected` on the PX4 side.
- Build/run inside ros2-autonomy:
  `colcon build --symlink-install --base-paths /ros2_ws/src
   --build-base /ros2_ws_build/build --install-base /ros2_ws_build/install
   --packages-select common_control`
  then `ros2 run common_control offboard_control_node --ros-args
  -p takeoff_height_m:=2.0 -p hover_seconds:=4.0`
  (source /opt/ros/humble, /opt/px4_ros2_ws/install, /ros2_ws_build/install).
- Two control-design lessons baked into the node (do not regress these):
  1. PX4 only considers the offboard signal present when the
     `OffboardControlMode` heartbeat AND a `TrajectorySetpoint` stream are
     both flowing — the node streams the takeoff setpoint from tick 0,
     including while disarmed in INIT.
  2. Mode-switch + arm are command-and-confirm with 1 Hz retry against
     `vehicle_status_v1`, never fire-and-forget (first attempt is often
     rejected by transient timing).

## Phase 2 result (2026-07-08)
- `ros2_ws/src/common_missions` exists and WORKS. Both missions flew the full
  cycle against SITL (arm → offboard → takeoff → waypoints → return → land →
  disarm), verified via the node's own state-transition logs:
  - **square** (`make mission` / `MISSION=square`): 4 m square, 4 waypoints,
    landed back at origin, "mission complete".
  - **survey** (`make mission MISSION=survey`): lawnmower over 8×6 m, 2 m
    lanes, 8 waypoints, returned to origin, "mission complete".
- Design: `common_control` gained a `WAYPOINTS` FlightState + `set_waypoints()`
  primitive; arrival is judged by position tolerance (`waypoint_tolerance_m`,
  default 0.5 m), NEVER elapsed time (sim runs slower than wall clock).
  `MissionBase` (in common_missions) subclasses OffboardControlNode and only
  supplies `build_waypoints()` + `declare_mission_parameters()`; a new mission
  is ~25 lines. `waypoint(north, east, height_m, yaw_deg)` helper hides the
  NED sign convention. Launch: `ros2 launch common_missions mission.launch.py
  mission:=square|survey` (OpaqueFunction maps mission name → console_script).
- Both missions can run back-to-back without restarting the sim — after the
  square mission disarmed, the survey mission re-armed and flew immediately
  (EKF2 stays converged).

## Bug found + fixed this session (was silently breaking the DDS bridge)
- `.env` had `UXRCE_DDS_PORT=8888          # inline comment`. docker compose
  strips the `# comment` but KEEPS the padding whitespace, so the value became
  `"8888          "`. MicroXRCEAgent rejects the padded port and exits at
  container boot → bridge never comes up → every `/fmu/*` topic exists but is
  silent → control node waits forever at `nav_state=0, arming_state=0`.
  This is the same "looks connected but no data" failure signature as the
  restart-wedge gotcha, but a different root cause. Fixed: (1) removed the
  inline comment from `.env`; (2) entrypoint now `tr -d '[:space:]'` on the
  port as a safety net; (3) documented as README known-issue 12. NOTE: the
  entrypoint change is baked into the image, so `make build` (or at least
  `docker compose build ros2-autonomy`) is required — a plain restart won't
  pick it up. Diagnose via `docker exec ros2-autonomy cat
  /var/log/microxrce_agent.log` (shows `'--port <value>' is required`).

## Phase 2.5 result (2026-07-08, later same day)
- **Host reality check**: the `.env` "no GPU on this host" note was WRONG.
  This dev machine is a ThinkPad W541 with a live X11 desktop (`DISPLAY=:0`)
  and an Intel HD 4600 iGPU (`/dev/dri/card1`) — enough for the real Gazebo
  GUI via Mesa. (There's also an NVIDIA Quadro K2100M present but unused —
  no `nvidia-smi`, so don't rely on it; Intel iGPU is the supported path.)
- **`make sim-gui`** now pops the actual Gazebo Harmonic window. Mechanism:
  `docker-compose.gui.yml` overlay forwards `/tmp/.X11-unix` + `/dev/dri` and
  sets `PX4_HEADLESS=0`; `Dockerfile.px4_sim`'s CMD branches on that to either
  `export HEADLESS=1` or `unset HEADLESS` before `make px4_sitl gz_<model>`.
  **Gotcha that cost real time**: PX4's `px4-rc.gzsim` checks `[ -z
  "$HEADLESS" ]` — a plain `HEADLESS=0` still counts as "set" and the GUI
  silently never launches (no error, just headless with no complaint). Must
  literally `unset` it. Documented as README known-issue 13.
- **World**: `docker/px4_sitl_worlds/empty.sdf` overlaid into
  `Tools/simulation/gz/worlds/` at build time (same overlay mechanism as the
  airframe `.post` param file — not a PX4 fork). PX4 keys every Gazebo topic
  as `/world/<PX4_GZ_WORLD>/...`, so the `<world name="...">` inside the SDF
  MUST equal `PX4_GZ_WORLD` ("empty") or the bridge silently subscribes to
  nothing. Confirmed via `gz topic -l | grep /world/`.
- **`sim_bringup` / `hw_bringup` packages built**, formalizing what README §2
  already described as the target architecture. Shared, transport-agnostic
  `common_missions/launch/autonomy.launch.py` (action=hover|mission,
  params_file=<path>) is included by both bringups — `common_control` /
  `common_missions` Python is untouched by any of this work.
  - `sim_bringup/launch/sim.launch.py` + `config/sim_params.yaml` — this is
    now what `make flight-test` / `make mission` actually run (colcon
    `--packages-up-to sim_bringup`), replacing the old direct `ros2 run` /
    `ros2 launch common_missions` calls. Mission/control params moved out of
    Python defaults and inline `-p` flags into this one YAML.
  - `hw_bringup/launch/hw.launch.py` + `config/hw_params.yaml` — untested
    STUB for Phase 4. Starts `MicroXRCEAgent serial` (vs sim's UDP), has a
    commented include-slot for `common_perception`/realsense-ros. Verified it
    launches cleanly and just retries "Serial port not found" forever with a
    fake device — correct behavior for hardware that doesn't exist yet, no
    crash.
- **Verified visually** (Gazebo GUI window, not just log lines): square
  mission (4 waypoints) and survey mission (8 waypoints) both flown through
  `sim_bringup`, watched live, landed/disarmed cleanly. Also reran plain
  `make sim` (headless) afterward to confirm no regression — server-only gz
  process, no GUI-start log line, no X11 errors.
- Sim speed varies run to run — the square mission's climb was ~13s this
  session vs ~77s in earlier sessions on the same laptop. Don't assume a
  fixed sim/wall-clock ratio.

## Mission params restructured to per-mission-name YAML sections (2026-07-08)
- User wants to tune missions by editing a config file, not code — and
  wants the file structured with the mission name as the top-level tag, so
  it reads as "one block per mission." Also flagged as groundwork for a
  future operator app that would pick a mission + edit these same values
  before launch (not built, just don't paint the config format into a
  corner that makes that awkward later).
- **`sim_params.yaml` / `hw_params.yaml` restructured**: replaced the old
  `/**:` wildcard block (applied identically to every node) with one
  top-level section per mission, keyed by that mission's actual ROS 2 node
  name (`offboard_control_node`, `square_mission`, `survey_mission`) — e.g.
  `square_mission:\n  ros__parameters:\n    side_length_m: 4.0`. This is
  **standard ROS 2 parameter-file syntax**, not a custom mechanism: each
  launched node automatically reads only the section matching its own name.
  Confirmed this works with a live test (bare, non-slash-prefixed node-name
  key correctly overrode a param) before touching the real files.
- **Zero Python/launch code changes** — `autonomy.launch.py` already just
  passes `parameters=[params_file]` through; it doesn't care how the file's
  sections are organized. This was purely a config-file restructure.
- Verified all four profiles load correctly through the REAL launch paths
  post-restructure (not just a standalone test): `ros2 launch sim_bringup
  sim.launch.py action:=hover`, `action:=mission mission:=square`,
  `action:=mission mission:=survey`, and `ros2 launch hw_bringup
  hw.launch.py` — checked via `ros2 param get /<node> <param>` matching each
  YAML's documented values exactly (sim: 4.0/2.0/0.5 etc.; hw: 2.0/1.5/0.7).
- README §7 rewritten with the YAML structure explained + example snippet +
  the future-app note, per [[drone-project-readme-style]].

## Params made REQUIRED (no code defaults) + build-ws split out (2026-07-08)
- User's follow-up pushback: even after the YAML restructure above,
  `offboard_control_node.py`/`square_mission.py`/`survey_mission.py` still
  had `declare_parameter(name, 4.0)`-style hardcoded literal defaults in
  Python. User doesn't want that — wants the params file to be the ONLY
  source of truth, and wants a clear error if something fails to initialize
  from it, not a silent fallback to a code default.
- **`OffboardControlNode._require_param(name)`** (new helper in
  `common_control/offboard_control_node.py`) is the fix: declares with
  `rclpy.Parameter.Type.DOUBLE` (a type, no value) instead of
  `declare_parameter(name, 4.0)`. Empirically confirmed (via a real broken
  params file, not guesswork) that rclpy Humble's `get_parameter()` raises
  `ParameterUninitializedException` the instant you read an unset
  type-only param — `_require_param` catches that and re-raises a
  `RuntimeError` with an actionable message after logging `.fatal(...)`:
  names the exact param, the node/section it should be in, and the file to
  check. Non-zero exit, `ros2 launch` reports the node as crashed. All 4
  control params (`offboard_control_node`) and all mission geometry params
  (`square_mission`, `survey_mission`) now go through this — zero numeric
  literals left as parameter defaults anywhere in the Python.
- Missions were also refactored to cache `self._side_length_m` etc. in
  `declare_mission_parameters()` instead of calling `get_parameter(...).value`
  again inside `build_waypoints()` — matches the pattern
  `OffboardControlNode.__init__` already used for its own params, one fewer
  parameter-service round trip.
- **Separate complaint, same session**: `make flight-test`/`make mission`
  were colcon-building the whole workspace on EVERY invocation — slow for
  the "tweak a YAML value, refly" loop this params work was supposed to
  enable. Fix: new `make build-ws` target (one-time colcon build);
  `flight-test`/`mission` now just check `/ros2_ws_build/install/setup.bash`
  exists (else tell you to run `build-ws`) and launch directly — no build
  step. This works because `--symlink-install` genuinely symlinks
  `data_files` (config YAML, launch files) all the way back to
  `ros2_ws/src/...` on the host — confirmed by `ls -la` following the
  symlink chain (install → build dir → source), then proved live: edited
  `side_length_m` to 9.0, ran the mission with ZERO colcon build in
  between, `ros2 param get` showed 9.0. Reverted to the real default (4.0)
  after.
- Verified end-to-end for real (not just unit-style): (1) normal path —
  full square mission flew correctly through `sim_bringup` with the
  refactored code, waypoints matched `side_length_m: 4.0`; (2) fail-loud
  path — a params file missing `side_length_m` under `square_mission:`
  produced the exact `[FATAL]` message above and exit code 1; (3) no-rebuild
  path — described above.
- **Self-inflicted incident, worth flagging**: mid-session, ran a bare
  `make sim` (headless) without checking `docker ps` first. Turned out
  ros2-autonomy was already up from ~15 min earlier with a `survey_mission`
  process still running (PID start time 09:06, well before this turn) —
  almost certainly the user's own `make mission MISSION=survey` from their
  stated plan to "test in gazebo." `make sim`'s headless config differs from
  `docker-compose.gui.yml`'s, so compose recreated JUST px4-sim to match
  (confirmed: px4-sim uptime was minutes vs ros2-autonomy's ~20) — i.e. this
  accidentally reproduced known-issue-5 (restart px4-sim alone while
  ros2-autonomy keeps running) UNDERNEATH the user's own in-progress test,
  and very likely caused two nodes (their survey_mission + this session's
  square_mission test) to briefly fight for control of the same vehicle.
  Cleaned up (killed both stray processes, full `docker compose down` +
  `make sim` together) before continuing. Lesson: **always `docker ps`
  before touching container state**, especially mid-session when the user
  said they'd be interacting with the stack themselves.

## Added max_velocity_m_s — cruise speed cap (2026-07-08)
- User: "the drone moves so fast" — with no velocity control at all, PX4's
  offboard position controller accelerates toward every setpoint at its own
  internal velocity limit (PX4 firmware default, not something this repo
  ever set), so a waypoint transition looks like a near-instant snap in the
  Gazebo GUI.
- New required param `max_velocity_m_s` in `OffboardControlNode` (same
  `_require_param` pattern as everything else — no code default, must come
  from the YAML). Added to every section of both `sim_params.yaml` (`1.0`)
  and `hw_params.yaml` (`0.5`, more conservative per the established
  sim-vs-hw pattern).
- **Mechanism**: `_capped_velocity_toward(x, y, z)` computes a unit vector
  from the current position to the target, scaled to magnitude
  `max_velocity_m_s`, and sends it as the `velocity` field on
  `TrajectorySetpoint` ALONGSIDE the existing `position` field — this is a
  standard PX4-ROS2 offboard technique (position + velocity feed-forward),
  not a PX4 firmware parameter change and not our own trajectory/waypoint
  interpolator. `_publish_setpoint` (used uniformly by every flight state —
  INIT, TAKEOFF, WAYPOINTS, HOVER) now always attaches this, so takeoff climb
  speed is capped too, not just waypoint-to-waypoint cruise.
- Verified for real: flew square (side_length_m: 3.0, max_velocity_m_s: 1.0)
  through the actual `sim_bringup` path — startup log confirmed
  `max_velocity=1.0m/s` loaded, and the first 3 m leg took ~27 s of
  wall-clock time (clearly rate-limited, not a snap-to-target jump). Startup
  log line now also prints `max_velocity=...` for visibility.
- Note: user had already hand-edited `sim_params.yaml`'s mission geometry
  before this change landed (`side_length_m: 3.0`, `area_length_m/width_m:
  5.0/5.0`, `lane_spacing_m: 0.5`) — preserved those, only added the new
  `max_velocity_m_s` lines alongside them. README §7 tables updated to match
  current file contents, not the original defaults.

## Phase 3 Milestone A result (2026-07-08, night): localization-source switch
- User wants localization source (GPS vs VIO) chosen ONCE before a mission
  starts — not a live in-flight switch (explicitly deferred) — mirroring
  real operation: fly outdoors → GPS, fly indoors → the RealSense D435i
  drives VIO. Full design record, including dead ends ruled out:
  `resource/phase3-gps-denied-localization-source.md` (new top-level
  `resource/` dir — durable, meant to be committed, unlike this file).
- **Critical finding that shaped everything**: PX4's uXRCE-DDS bridge — used
  for literally everything else in this stack — cannot set PX4 parameters at
  all in this pinned v1.17.0 (exhaustive search of `dds_topics.yaml`: zero
  `Parameter*` topics bridged; `VEHICLE_CMD_DO_SET_PARAMETER` exists in the
  message set but PX4's own `Commander.cpp` doesn't handle it). Had to add a
  small MAVLink `PARAM_SET` side-channel (`pymavlink`) — PX4 SITL already
  exposes a MAVLink port independent of the DDS agent, so no PX4 rebuild.
- **New `common_perception` package** (the 3rd `common_autonomy` package
  IMPLEMENTATION_PLAN.md's architecture diagram always anticipated):
  - `set_localization_source` — one-shot script (not an rclpy Node — a
    single imperative MAVLink transaction). Sets `EKF2_GPS_CTRL`/
    `EKF2_EV_CTRL` only, NEVER `EKF2_HGT_REF` (that one needs a reboot;
    disabling GPS makes PX4's own automatic height fallback pick baro
    instead, no reboot needed — full reasoning in the resource doc).
  - `frame_transforms.py` + `loopback_odometry_bridge.py` — the Milestone A
    fake-VIO stand-in: loops PX4's own `/fmu/out/vehicle_odometry` back in
    as `/fmu/in/vehicle_visual_odometry`, proving the whole mechanism with
    zero camera/OpenVINS work. `openvins_odometry_bridge.py` (Milestone B,
    not started) will be a drop-in replacement publishing the same topic.
  - `sim_bringup`/`hw_bringup` gained `localization_source`/`mavlink_url`
    launch args, sequenced via `RegisterEventHandler(OnProcessExit(...))` so
    the mission only starts after the switch confirms. `Makefile` gained
    `LOCALIZATION ?= gps` (mirrors `MISSION ?=`):
    `make mission MISSION=square LOCALIZATION=vision`.
- **Two real bugs found and fixed via live testing, not guessed**:
  1. MAVLink's `PARAM_SET`/`PARAM_VALUE` wire format packs every value into
     a float32 slot regardless of real type — for PX4's INT32 params (e.g.
     `EKF2_GPS_CTRL`, `param_type=6`) the bytes must be **reinterpreted**,
     not numerically cast. A naive `float(7)` reads back as `~9.8e-45`, not
     `7`. Fixed with `struct.pack`/`unpack` round-tripping in both directions
     — verified live against running SITL before trusting the code (a naive
     first version would have made every confirmation permanently fail).
  2. `ExecuteProcess` calls a raw OS subprocess — console_scripts in this
     repo install to `install/<pkg>/lib/<pkg>/`, resolved via the ament
     index (how `ros2 run`/`ros2 launch` Node actions find them), NOT on
     plain `$PATH`. `cmd=['set_localization_source', ...]` failed with
     `FileNotFoundError`; fixed to `cmd=['ros2', 'run', 'common_perception',
     'set_localization_source', ...]`.
  3. Also found empirically: the correct MAVLink connection string is
     `udpin:0.0.0.0:14540` (bind and listen), not a naive `udp:host:port`
     connect-string — PX4's "offboard" MAVLink instance (`px4-rc.mavlink`)
     listens on 14580 and SENDS unsolicited to 14540, so the companion
     computer must be the listener.
  4. **Self-inflicted, caught by watching a suspiciously slow test**: edited
     `pymavlink` into the Dockerfile source, but only ever `pip install`ed it
     ad hoc into the *running* container for live testing — never actually
     rebuilt the image. A later `docker compose down`/`up` (needed to clear
     stray processes, see below) wiped that ad hoc install, and
     `set_localization_source` died with `ModuleNotFoundError`. Fixed by
     actually running `docker compose build ros2-autonomy`. Lesson: an ad
     hoc `pip install`/`apt install` inside a running container is not the
     same as baking it into the image — always verify via a real rebuild +
     restart before calling a Dockerfile change done.
  5. **Also self-inflicted**: background test processes from `ros2 launch`
     don't reliably exit even after their child rclpy node cleanly finishes
     (`rclpy.shutdown()` on mission DONE) — the launch wrapper can linger.
     Ran 3 sequential tests without confirming each one's processes fully
     exited, ended up with 3 concurrent offboard controllers (2 stale +1
     new) all fighting over the same simulated vehicle, which explained an
     anomalously stalled mission. Always `ps aux | grep ros2 launch` before
     trusting a "slow" flight is just slow-and-not-contended.
- **Verified end-to-end** (after both bugs above were fixed, on a freshly
  rebuilt image + fresh container pair, zero contending processes):
  - `LOCALIZATION=gps` — regression check, unchanged behavior, full square
    mission flies and lands normally (`EKF2_GPS_CTRL=7`/`EV_CTRL=0`, a
    no-op vs pre-Phase-3 behavior).
  - `LOCALIZATION=vision` — confirmed via `estimator_status_flags`:
    `cs_gnss_pos: false`, `cs_gnss_vel: false` (GPS genuinely off),
    `cs_ev_pos: true`, `cs_ev_vel: true` (vision genuinely fusing),
    `cs_ev_hgt: false` (height correctly on baro, as designed). Both
    `square` and `survey` missions flew their full waypoint sequences and
    landed/disarmed normally while fully GPS-denied — proof that
    `common_control`/`common_missions` are completely unmodified and
    unaware of the source.

## Phase 3 Milestone B result, part 1 (2026-07-09) — plumbing verified, VIO quality NOT flight-safe yet (SUPERSEDED — see "part 2" below, root cause found and fixed same day)
- All of Milestone B's design (`ros_gz_bridge` config, OpenVINS built from
  source, `vio_test.sdf` world, `openvins_odometry_bridge.py`) was already
  written out in a prior uncommitted working-tree state when this session
  picked it up — `DEVELOPMENT_STATUS.md` just hadn't been updated to say so.
  This session's job was verifying it actually works, not writing it.
- **Build verified**: both `docker compose build px4-sim` and
  `ros2-autonomy` complete clean (fully cached from earlier work — layers
  include `ros-humble-ros-gzharmonic-bridge` from OSRF's apt repo and
  `ov_core`/`ov_init`/`ov_msckf`/`ov_eval` built from source into
  `/opt/openvins_ws`). `colcon build` of `ros2_ws/src` (now including
  `common_perception`) succeeds.
- **Bridge topic names verified live**: booted `px4-sim` with
  `PX4_GZ_WORLD=vio_test` and confirmed via `gz topic -l` that all three
  `gz_topic_name`s hardcoded in `config/ros_gz_bridge.yaml` (RGB image,
  camera_info, and the flight-controller's own `imu_sensor`) exist
  byte-for-byte under the live `/world/vio_test/model/x500_depth_0/...`
  namespace — no drift between the config and the running world.
- **OpenVINS genuinely initializes against `vio_test.sdf`'s colored boxes**:
  ran `ros2 launch common_perception vio.launch.py` standalone — feature
  tracker goes from `not enough feats`/`disparity 0.000` to `successful
  initialization` in ~1-5s once the drone has any motion at all, and
  `/fmu/in/vehicle_visual_odometry` (via `openvins_odometry_bridge`) streams
  at ~31 Hz. `empty.sdf` genuinely would NOT work here (no trackable
  corners) — the dedicated world was a real, correct requirement, not
  over-engineering.
- **GPS-denied fusion confirmed live end-to-end**: ran `make flight-test`
  equivalent with `LOCALIZATION=vision VIO_BACKEND=openvins`. Localization
  switch confirmed `EKF2_GPS_CTRL=0` / `EKF2_EV_CTRL=5`. Mid-hover,
  `px4-listener estimator_status_flags` showed `cs_gnss_pos: False`,
  `cs_gnss_vel: False`, `cs_ev_pos: True`, `cs_ev_vel: True` — this is the
  real OpenVINS estimate driving EKF2, not the Milestone A loopback stand-in,
  and GPS is genuinely excluded, not just nominally disabled.
- **NEW PROBLEM FOUND — VIO estimate diverges mid-flight, not flight-safe**:
  during the same hover test, OpenVINS's own `p_IinG` (position-in-global)
  climbed roughly linearly for the whole hover — from near-zero at init to
  `(2.15, 4.65, 1.39)` metres by the time logging was cut off — while the
  vehicle was commanded to hold a fixed point. `vehicle_local_position`
  confirmed this reached PX4: `x=-7.3, y=-24.9` metres and climbing, with
  `xy_reset_counter: 7` and `vxy_reset_counter: 6` (EKF2 repeatedly
  fast-resetting against the diverging vision input, not settling). The
  drone never landed on its own — commanding `AUTO_LAND` did nothing (still
  climbing after), and it had to be force-disarmed
  (`px4-commander disarm -f`) to bring it down. Root cause not yet
  diagnosed — leading hypotheses: (1) feature starvation once the vehicle
  moves away from `vio_test.sdf`'s prop ring (a stationary hover shouldn't
  need this, so more likely closer to (2)); (2) `config/openvins/*.yaml`
  calibration (adapted by hand from `config/rs_d455`, never verified against
  the actual simulated `IMX214` camera's real intrinsics/extrinsics) is
  wrong enough to bias scale/velocity, causing steady dead-reckoning drift
  once vision updates can't fully correct it. This is real tuning work, not
  a plumbing bug — matches what was anticipated below before this session
  started but is now an OBSERVED, reproduced failure, not a hypothetical.
  **Do not fly `VIO_BACKEND=openvins` unattended (e.g. `make mission`)
  until this is root-caused — hover alone already diverges within ~1 minute.**
- Also reconfirmed the documented "background ros2 launch lingers after its
  parent process dies" gotcha (see below) — `timeout 180 docker exec ...`
  killed the outer shell but `ros2 launch`, `offboard_control_node`,
  `parameter_bridge`, and `run_subscribe_msckf` all kept running and had to
  be `kill -9`'d individually by PID before the vehicle would stay disarmed.

## Phase 3 Milestone B result, part 2 (2026-07-09, same day) — root cause found, fixed, and retested clean

**Root cause of the part-1 divergence, confirmed (not just hypothesized):**
`config/openvins/kalibr_imucam_chain.yaml`'s `T_cam_imu` rotation was
identity. That's wrong for any forward-facing camera on an FLU-body vehicle
— a camera's SDF `<pose>` is declared in the link's own FLU-like body
convention, but the pixel-projection ("optical") frame OpenVINS's pinhole
model assumes is always +Z-forward/+X-right/+Y-down. Those are different
frames regardless of translation, and the earlier design doc reasoning ("the
whole chain is pure translation, no rotation anywhere") was true of the
*SDF pose chain* but false of the *body→optical* relationship — two
different rotations that got conflated. Correct rotation:
`R_cam_imu = [[0,-1,0],[0,0,-1],[1,0,0]]` (body +X→cam +Z, body +Y→cam −X,
body +Z→cam −Y).

**Fixed alongside a second, independently-motivated change**: swapped the
simulated camera from the stock Gazebo `x500_depth` model's `OakD-Lite` to
a locally-authored `d435i` model (`docker/px4_sitl_models/d435i/`), so sim
flies the same physical camera (Intel RealSense D435i) Phase 4 hardware
will use — not a different camera wearing matched topic names. Per-repo
overlay only: NOTHING in the PX4-Autopilot checkout was edited — the new
model directory and a locally-owned copy of `x500_depth/model.sdf`
(pointing at `model://d435i` instead of `model://OakD-Lite`) are `COPY`'d
into the built image by `Dockerfile.px4_sim` post-clone, the exact same
overlay mechanism already used for `vio_test.sdf`/the airframe `.post`
override. The new model also gives the camera a real, working onboard IMU
(the old `OakD-Lite`'s `camera_imu` sensor name was a dangling
zero-publisher topic, worked around before by bridging the flight
controller's IMU instead — a sim/real topic mismatch, since real
`realsense-ros` `/camera/camera/imu` is the CAMERA's own IMU). Co-locating
the new IMU with the color sensor also zeroed out `T_cam_imu`'s translation
component, leaving only the rotation fix above as the nontrivial part of
the extrinsic. Full derivation: `config/openvins/kalibr_imucam_chain.yaml`
header, `docker/px4_sitl_models/d435i/model.sdf` header, and
`resource/phase3-gps-denied-localization-source.md` finding 6.

**Retested — same test as part 1 (stationary-position hover,
`LOCALIZATION=vision VIO_BACKEND=openvins`), actively monitored this time at
~3s resolution through the whole flight rather than checked only at the
end:**
- Climb was fast and clean (reached ~2m in ~15s, vs. part 1's pathological
  51s) — a sign the estimate was stable enough that the controller wasn't
  fighting noisy/wrong position feedback.
- Horizontal position stayed bounded the entire flight — wobbled within
  roughly ±30cm during climb/hover/descent (normal mono-VIO noise), never
  showed the linear runaway-growth pattern from part 1.
- Landed and **auto-disarmed on its own** — no `AUTO_LAND` stall, no manual
  `px4-commander disarm -f` intervention needed this time. Final position
  error after the full cycle: `x=0.067, y=-0.302` (~30cm), vs. part 1's
  `x=-7.3, y=-24.9` (~26m) before it had to be forced down.
- `xy_reset_counter` incremented only twice total during the whole flight
  (vs. climbing continuously through 7+ in part 1) — EKF2 settling on the
  vision input instead of repeatedly rejecting it.

**Caveat — this is ONE successful flight, not a proven-robust result.**
Only a stationary hover was retested (same test as part 1, for a clean
before/after comparison), not `square`/`survey` missions with real lateral
motion and yaw changes, and not repeated across multiple runs. Before
trusting `VIO_BACKEND=openvins` for `make mission` unattended:
1. Re-run this same hover test 2-3 more times (SITL is nondeterministic
   run-to-run) to confirm the fix isn't a lucky single sample.
2. Fly `square`/`survey` with `VIO_BACKEND=openvins` — real translation and
   yaw exercise the extrinsic rotation fix far more than a stationary hover
   does; if the rotation were still subtly wrong, lateral motion is where
   it would show up first.
3. `EKF2_EVP_NOISE`/`EKF2_EVV_NOISE`/`EKF2_EVA_NOISE` tuning (mentioned in
   `openvins_odometry_bridge.py`'s covariance-passthrough comment) is
   still untouched — fine for now since the fix addressed the actual root
   cause rather than papering over it with trust-it-less noise params, but
   worth revisiting if a mission-profile test shows noisier tracking than
   this hover did.
- `make flight-test`/`make mission` no longer colcon-build on every
  invocation (fixed earlier — see "build-ws split" below). Auto-running
  `build-ws` once from the entrypoint on first container boot is still
  optional polish, not done.
- The real RealSense D435i hardware path (Phase 4) is a little less blocked
  now than before — sim and real now share the same camera identity, not
  just matched topic names — but hardware bring-up itself is still fully
  untested (`hw_bringup` remains an untested stub, see below).

## Phase 3 Milestone B result, part 3 (2026-07-09, same day) — real mission tested, SECOND extrinsics bug found + fixed, NEW post-landing drift gap found (open)

**Target hardware this is all in service of**: real Pixhawk 6C (running PX4
v1.17.0, same version as SITL) + Intel RealSense D435i (real VIO source,
matching the sim swap above) + Orange Pi 5 Plus (Ubuntu 22.04 companion
computer, matching `ros2-autonomy`'s own Humble/22.04 base). Keep this
sim→real path in mind for every choice here — anything that only works
because it's SITL (e.g. relying on `docker compose down/up` to reset state)
needs a real equivalent before Phase 4, not a shrug.

**Flew `make mission MISSION=square LOCALIZATION=vision VIO_BACKEND=openvins`
for real** (part 2 only retested the stationary hover). First attempt, on a
sim stack that had already run 2 prior tests without a full restart, failed
to arm at all — `pre_flight_checks_pass: False`, health_report decoded to
`local_position_estimate`/`global_position_estimate` failing, root-caused to
`/camera/camera/color/image_raw` publishing ZERO messages even though the
Gazebo-side sensor, `ros_gz_bridge`, and OpenVINS were all alive — a stale
ROS graph after repeated launches on a long-lived container, matching this
repo's documented "looks connected but no data" failure signature. Fixed
(as usual) by a full `docker compose --profile sim down && up`, not a
partial restart. **Operational lesson for real hardware**: don't just
re-launch the mission node on top of a ROS graph that already ran a prior
flight — the perception pipeline (bridge + OpenVINS) needs a clean restart
per flight until/unless this is root-caused further, and a real Orange Pi
has no `docker compose down` equivalent — this needs a proper fix (e.g. a
supervisor that tears down and relaunches the perception nodes between
missions) before Phase 4, not just "remember to reboot."

**On a fresh stack, the mission flew all 4 waypoints with the part-2 fix in
place — extremely accurately, well within 30cm of every target, zero EKF
resets during waypoint navigation** — real lateral motion and yaw, the
strongest test yet of the rotation fix. Then, during the fast `AUTO_LAND`
descent (`vertical_movement`/`horizontal_movement`/`rotational_movement`
all `True` simultaneously — the most aggressive combined dynamics of the
whole flight), the estimate diverged for real:
`vehicle_local_position` ran to `x=11.4, y=-10.8, z=4.97` (physically
impossible — that's underground) before a forced disarm.

**Root cause #2, confirmed empirically, not guessed**: `run_subscribe_msckf`
has `calib_cam_extrinsics: true`, meaning OpenVINS treats
`kalibr_imucam_chain.yaml`'s `T_cam_imu` as a mere *seed*, not a fixed
value, and refines it online. Grepping the flight's OpenVINS debug log for
`cam0 extrinsics` showed the estimator converging within seconds to, and
holding rock-stable for 261+ consecutive log lines (the entire flight), a
quaternion of `(-0.5, 0.5, -0.5, 0.5)` — which decodes to EXACTLY the
**transpose** of the rotation matrix part 2 hand-derived and seeded. The
part-2 fix had the right geometric insight (camera-optical vs. body-FLU
axes really are different, identity really was wrong) but got the
**direction** of that rotation backwards. Waypoint navigation flew
accurately anyway because the online calibrator silently corrected the bad
seed to the right answer — but under the landing phase's aggressive
combined dynamics, that same online correction couldn't keep up and the
filter diverged for real. **Fixed** by seeding the actual converged value
directly (`R_cam_imu = [[0,0,1],[-1,0,0],[0,-1,0]]`,
`kalibr_imucam_chain.yaml`) and setting `calib_cam_extrinsics: false`
(`estimator_config.yaml`) to freeze it — not just fixing this one instance,
but removing "online extrinsics correction lags under aggressive dynamics"
as a failure class entirely, since the value is now known-good from a real
flight rather than hand-derived and hoped-correct.

**Retested on ANOTHER fresh stack with both fixes in place: all 4 waypoints
flown accurately again, AND the fast `AUTO_LAND` descent that diverged
before this time stayed controlled** — `vz` rose and fell smoothly
(0.36→0.72→0.33 m/s) with no discontinuous jumps, nothing resembling the
`x=11.4,y=-10.8` runaway. This is real, direct evidence the direction fix
resolved the aggressive-dynamics divergence specifically.

**NEW PROBLEM FOUND — post-landing position/velocity estimate drift,
currently unfixed, blocks auto-disarm**: after the controlled descent
above, PX4's OWN estimate kept "sinking" past ground level at a suspiciously
*constant* rate (`vz` pinned at ~0.40 m/s for many seconds straight, `z`
growing linearly past +2m — physically impossible) and drifted
horizontally too, while `vehicle_land_detected.landed` stayed `False`
forever, blocking auto-disarm. Checked Gazebo's OWN ground-truth model pose
directly (`gz topic -e -t /world/vio_test/pose/info`) to settle whether the
real vehicle was doing something dangerous: **it was not** — ground truth
showed the model essentially motionless at `z≈0`, resting normally. The
divergence is entirely in PX4's estimate, not the physical vehicle. Had to
force-disarm again (`px4-commander disarm -f`), same as part 1, but this
time the vehicle was never actually at risk.
- **Root cause, strongly suspected from the config itself (not yet
  independently verified via a second test)**: `estimator_config.yaml` sets
  `zupt_only_at_beginning: true`, which by its own header comment
  deliberately disables OpenVINS's zero-velocity-update safety net "once
  real flight starts" — precisely so it doesn't interfere with genuine
  flight. But that means once landed, there is NO ZUPT catching the vehicle
  back to zero velocity while it sits stationary, and ordinary IMU bias
  (visible in the same log as a small but nonzero `ba`) free-integrates
  into a growing phantom velocity/position with nothing to correct it —
  especially plausible near-ground, where a downward-tilted or
  close-range/low-texture camera view post-touchdown may also be giving
  OpenVINS materially worse visual corrections than mid-flight, though this
  wasn't independently confirmed this session (no feature-count logging was
  captured for the post-landing window specifically).
- **Why this matters for Phase 4, not just SITL polish**: a real Pixhawk
  6C's own land-detector and disarm logic almost certainly has the same
  shape of dependency on a sane velocity estimate. If the real onboard
  D435i+Orange Pi VIO pipeline has this same gap, a real landing could
  fail to auto-disarm, or worse — depending on what a real EKF2/controller
  does with a phantom nonzero velocity estimate while sitting on the
  ground (fighting a false "still descending" belief with motor commands
  is a real prop-strike/tip-over risk in a way a simulated version isn't).
  **This should be closed before any real hardware flight, not deferred to
  "tune later."**
## Phase 3 Milestone B result, part 4 (2026-07-09, same day) — post-landing-drift fix attempted, caused a WORSE regression, reverted

**Tried the obvious fix for part 3's open item**: `estimator_config.yaml`
had `zupt_only_at_beginning: true`. Verified by reading `VioManager.cpp`
directly that this permanently latches OFF OpenVINS's runtime zero-velocity
correction the first time the vehicle ever moves (`has_moved_since_zupt`),
which is exactly why nothing was catching drift post-landing. Flipping it
to `false` matches both OpenVINS's own default AND `rs_d455` (the reference
config this file was adapted from) — looked like a safe, well-precedented
one-line fix.

**It was not safe — confirmed by an actual flight, not just reasoning about
it.** With `zupt_only_at_beginning: false`, the vehicle never left the
ground at all on the very next test: "Armed and offboard — climbing" logged
normally, but `z` stayed at ~0 for 15+ seconds while the OpenVINS log showed
64+ consecutive `[ZUPT]: accepted |v_IinG| = 0.001` lines. **Root cause is
structural, not a bad parameter value**: any takeoff necessarily starts
from `v=0`, so a ZUPT gate with no "only at the very beginning" restriction
fires at the exact instant thrust first increases, "confirms" zero velocity
as ground truth, and creates a self-reinforcing lock (EKF believes
stationary → real motion stays too small to move disparity past threshold
→ ZUPT re-fires next frame → repeat). OpenVINS's existing flags support
"ZUPT only pre-flight, then latched off forever" or "ZUPT any time
conditions are met" — but not the policy this project actually needs,
"ZUPT before flight AND after landing, but never at the moment flight
begins." That policy doesn't exist as a config flag; this is a genuine
capability gap in the upstream tool, not a tuning mistake on this repo's
part.

**Reverted `zupt_only_at_beginning` back to `true`** (rebuilt, fresh
stack, retested): takeoff, all 4 waypoints, and the previously-diverging
fast descent all confirmed working again, matching part 3's proven-good
mission5 run byte-for-byte in behavior. **Post-landing drift is back too**,
as expected — `vz` oscillated (0.71 → 0.03 → 0.71 m/s) without settling and
`landed` never went true, needing another `px4-commander disarm -f`. This
is the known, still-OPEN issue from part 3 — unfixed, not worse, not
better. Net result of this part: confirmed a real dead end (don't try
`zupt_only_at_beginning: false` again without also solving the takeoff-lock
problem it causes) and eliminated one candidate fix, nothing more.

**Updated plan for actually closing the post-landing drift gap**, now that
the OpenVINS-side option is understood to be a dead end without much
deeper surgery (e.g. patching `VioManager.cpp` to gate ZUPT on something
smarter than "hasn't moved since start," which is out of scope for a config
change):
1. **Leading candidate, not yet tried**: fix this at the `common_control`
   level instead of the VIO level. The mission's own state machine already
   knows it commanded `AUTO_LAND` — after a reasonable timeout (or a
   low-throttle/near-zero-commanded-velocity heuristic, independent of the
   diverging position estimate), it could issue an explicit disarm command
   directly, the same command-and-confirm pattern already used for arming,
   rather than waiting on PX4's `land_detected` (which depends on exactly
   the estimate that's drifting). This is a deliberate, considered change
   to `common_control` — previously verified completely unmodified through
   all of Phase 3 — so it's flagged here for a decision rather than done
   unilaterally under this bug.
2. Also worth checking independently: whether the post-landing drift is
   partly a vision-quality problem specific to being very close to the
   ground (motion blur, near-field focus, losing the `vio_test.sdf` prop
   ring from a low viewing angle) rather than purely a "no ZUPT" gap — this
   session did not capture feature-count/tracking-quality logs for the
   post-landing window specifically to distinguish the two.
3. A PX4-side mitigation is also possible in principle (e.g. switching
   `EKF2_EV_CTRL` back toward a GPS/none default once `AUTO_LAND` is
   commanded, independent of vision health) but wasn't investigated this
   session.

## Phase 3 Milestone B result, part 5 (2026-07-09, same day) — a SECOND post-landing-drift fix attempted, ALSO caused a regression, ALSO reverted

**Tried a different angle on part 4's open item**: instead of touching
OpenVINS's ZUPT (the part-4 dead end), added a "stillness override" to
`openvins_odometry_bridge.py` itself — subscribe directly to the raw
`/camera/camera/imu` stream (ground-truth sensor data, not a filtered
estimate), detect sustained low gyro rate + accel magnitude near 1g over a
dwell window, and while "still," republish zero velocity and hold position
frozen instead of passing OpenVINS's own output through. The theory: this
can't reproduce part 4's takeoff-lock, because raw accel deviates from
gravity by far more than the threshold the instant real thrust is applied,
so the override should disengage immediately at takeoff and only matter
post-landing.

**That theory was wrong, confirmed live, same session.** The override
stayed engaged for **23 seconds** after "Armed and offboard — climbing" was
logged — `z` sat at ~0.05 (essentially the pad) the whole time. Root cause:
accelerometer-based "is moving" detection has a fundamental blind spot —
it senses *acceleration*, not *velocity*. This mission's `max_velocity` is
capped at 0.2 m/s; once the vehicle reaches that cruise speed (which a
well-tuned controller does quickly), sustained vertical acceleration drops
back near zero even though the vehicle IS moving, because constant-velocity
motion produces no net accelerometer deviation from gravity. Same blind
spot on the gyro side — a level, non-rotating vertical climb produces near-
zero angular rate too. So "moving" only shows up as a brief transient at
the very start of a velocity change, not throughout a slow, smooth climb —
and 0.2 m/s is slow and smooth almost by definition. When the override
finally did disengage (something eventually pushed a transient over
threshold — a mode-switch settling, a minor correction, not identified
precisely), the vehicle had ALREADY climbed and drifted for 23 seconds
while EKF2 was being told "you have zero velocity, you are exactly where
you started" — a real discrepancy had built up between belief and reality,
and it showed up as genuine divergence right after "Reached takeoff
height": `x=-7.8, y=-2.6` instead of the expected ~0,0. Had to force-disarm
again.

**Reverted** `openvins_odometry_bridge.py` and `package.xml`'s
`sensor_msgs` dependency back to plain OpenVINS passthrough (rebuilt,
confirmed clean). This is the SAME state as after part 4 — post-landing
drift still open, takeoff/waypoints/descent still known-good, no new
regression left behind.

**Bigger-picture lesson from parts 4 AND 5, worth internalizing before a
third attempt**: this project's missions are deliberately flown SLOW
(`max_velocity` capped well under 1 m/s, a real, considered safety choice
per this repo's own commit history — see "velocity cap" in recent commits).
That safety choice is exactly what breaks any "detect stillness from
motion-magnitude thresholds" approach — TWO independent implementations of
that idea (OpenVINS's built-in ZUPT, and a hand-rolled raw-IMU version)
both failed the same way, for related reasons (one via velocity threshold,
one via acceleration threshold), because a deliberately slow mission
profile sits too close to "looks stationary" by almost any inertial
measure. **The fix almost certainly needs to come from a signal that
ISN'T inertial-motion-magnitude-based** — the leading candidate from part 4
remains the most promising: `common_control`'s own mission state machine
already unambiguously KNOWS whether it commanded a takeoff or a landing
(it doesn't need to infer "am I moving" from noisy sensor thresholds at
all), so an explicit disarm-after-landing-timeout there sidesteps this
entire class of problem rather than trying to out-tune it. Do not attempt
a third motion-threshold-based fix without addressing this structural
mismatch first.

## Phase 3 Milestone B result, part 6 (2026-07-09, same day) — common_control fallback-disarm IMPLEMENTED and WORKS AS DESIGNED, but exposed a MORE SEVERE, separate, still-unfixed problem

**Implemented part 5's leading candidate**: `common_control/offboard_control_node.py` now subscribes to `/fmu/out/vehicle_land_detected` and, in `FlightState.LAND`, tracks `has_low_throttle` specifically (not the aggregate `landed` flag, which also requires `!horizontal_movement` — exactly the estimate-derived signal that drifts). `has_low_throttle` is confirmed, by reading `MulticopterLandDetector.cpp` directly, to be a pure actuator thrust-setpoint comparison — genuinely independent of the position/velocity estimate. Design is safety-first, not just a convenience filter:
- Requires `has_low_throttle` sustained continuously for `land_disarm_low_throttle_dwell_s` (new required param, no code default — sim: 3.0s, hw: 4.0s) before ever acting; any single high-throttle sample resets the dwell to zero.
- `land_disarm_max_timeout_s` (new required param — sim: 60s, hw: 90s) is a CEILING on how long to wait for that real signal, never a trigger by itself — if exceeded without the dwell condition ever being met, this logs a loud `.error()` and falls back to exactly today's known-safe behavior (stay armed, require manual `px4-commander disarm -f`), rather than ever guessing based on elapsed time. **This was a deliberate rejection of a naive "disarm after N seconds" design** — a stuck descent (wind, payload, mechanical issue) mid-air plus a blind timeout-disarm would drop the vehicle from height; the whole point of gating on `has_low_throttle` is to never do that.

**Real bug caught during first test, fixed**: the explicit disarm command was silently `MAV_RESULT_TEMPORARILY_REJECTED` by PX4 every time (confirmed via `vehicle_command_ack`) — `Commander::disarm()` requires `forced=true` OR its own `landed`/`maybe_landed` flags, which turned out to have their own separate dependency chain (`ground_contact` requires `_in_descend`, itself gated on a live trajectory setpoint — see `MulticopterLandDetector.cpp`) that was ALSO stuck false, independent of `has_low_throttle`. Without `forced=true`, PX4 was silently second-guessing this fallback's own (more careful, estimate-independent) judgement using the very broken signal it exists to route around — the fallback would have been a permanent no-op. Fixed by adding `param2=21196.0` (PX4's documented force-arm/disarm magic value, confirmed from `Commander.cpp`'s own CLI handler for `disarm -f`) — the exact same mechanism this project's manual escape hatch has used all session.

**Confirmed the safety design works as intended, the hard way — by watching it happen live, not by reasoning about it in the abstract**: across two full mission retests, the fallback never fired while `has_low_throttle` was false, INCLUDING once during a genuinely severe episode (below) where the vehicle was actively, erroneously climbing to real altitude. It correctly waited. That is the fallback doing exactly its job — refusing to guess.

**What it caught, and could not have prevented: a MORE SEVERE, independent problem, found live in these same two retests.** In both, sometime after "All waypoints reached — landing" / during `AUTO_LAND`, OpenVINS's fed estimate diverged badly enough that PX4's OWN `AUTO_LAND` controller — not this project's code, PX4's internal autonomous landing logic, chasing its own EKF2 state — caused the PHYSICAL vehicle to fly erratically. Checked via Gazebo's own ground-truth pose each time (independent of the diverging estimate), not inferred:
- Run 1 (mission7): vehicle drifted horizontally during a chaotic descent — but DID land safely (`z≈0.03` ground truth, `freefall: False`) — just ~8-9m off the intended spot.
- Run 2 (mission8): vehicle initially landed accurately (`z≈0.10-0.19` ground truth) but then, with `has_low_throttle: False` and `freefall: False` throughout — meaning it was under active, if wrong, thrust, not falling — CLIMBED AWAY to real altitude: confirmed ground-truth readings of z=3.8m, 7.5m, 9.2m, then still 11.1m and rising, while drifting to y=38.7m from origin, before intervention. This was not this project's code commanding that (once `AUTO_LAND` is active, `common_control` stops publishing any offboard setpoints — see the LAND state — PX4 flies its own internal profile from here). **PX4 itself, trusting its own drifted EKF2 state, flew the vehicle away.**

**Recovered by switching localization back to GPS mid-incident** (`ros2 run common_perception set_localization_source --source gps ...` — this project's own Milestone A mechanism, working exactly as designed as an emergency lever, not just a launch-time choice) — the XY estimate snapped back to something sane almost immediately, though the vehicle kept climbing a little further before it responded, and elevation was ultimately halted with a manual `px4-commander disarm -f` once judged not worth letting SITL continue (harmless in sim; this exact call would need to be made very differently — and much sooner — with different tools on real hardware, since force-disarming at real altitude is a crash).

**This changes the overall risk assessment of Milestone B, not just the disarm-mechanism story.** The `has_low_throttle` fallback disarm is real, working, and worth keeping — it correctly resolves the ORIGINAL documented bug (vehicle safely landed, PX4 stuck armed because its estimate wrongly believes otherwise — parts 3/5's finding) without ever risking a bad disarm. But it is not, and cannot be, a fix for THIS session's new finding: **OpenVINS's estimate can, at least intermittently (2 for 2 in tonight's retests, vs. clean landings in earlier parts 2/3's mission3/mission5/mission6 runs), become bad enough post-landing-command to make PX4's own autonomous `AUTO_LAND` unsafe, not just "stuck armed."** That is a VIO-estimate-quality problem, upstream of anything `common_control` can fix by being smarter about when to disarm. **Do not trust `VIO_BACKEND=openvins` for any unattended flight — sim or real — until this is root-caused.** Leading unknowns, not yet investigated:
- Whether this is the SAME root cause as parts 3-5's "resting but estimate drifts" bug just manifesting more severely this session (worse tracking quality this particular run — SITL isn't perfectly deterministic run-to-run, as already documented elsewhere), or a genuinely different failure mode specific to `AUTO_LAND`'s own controller behavior (distinct from the offboard position controller `common_control` uses for the rest of the flight).
- No feature-count/tracking-quality logging was captured for the post-landing-through-divergence window in either incident — first thing to add before debugging further.
- Whether freezing `calib_cam_extrinsics` (part 3's fix) interacts badly with severely degraded near-ground tracking in some way not yet understood — speculative, not confirmed.

## Phase 3 Milestone B result, part 7 (2026-07-09, same day) — root cause found and FIXED, confirmed clean on 2 consecutive full mission retests

**Root-caused part 6's in-flight divergence, not just worked around it.** Three independent, verified contributing factors, each fixed:

1. **IMU excitation starvation from the slow cruise speed.** Grepped OpenVINS's own debug log (`ba = ...` line) across the whole flight in a diverged run (mission8) vs. an earlier clean one: accelerometer bias was `-0.0000` at init and STILL ACTIVELY GROWING at landing time (`-0.0066 → -0.0318 → -0.0174,0.0428`, never settling) in the diverged run at `max_velocity_m_s: 0.2`, vs. converged/stable (`0.0039,-0.0168,-0.0304`, identical across 5 consecutive lines) in an earlier successful run at `1.0`. Monocular VIO scale/bias is only observable from real acceleration events — a >90s flight at a near-constant 0.2 m/s starves the filter of exactly that for the whole mission, so bias/scale error was already large and unconverged by the time landing's more dynamic maneuvering hit it. **Fixed**: raised `square_mission`/`survey_mission`'s `max_velocity_m_s` from `0.2` to `0.8` in `sim_bringup/config/sim_params.yaml` (still below the hover profile's `1.0`, kept as cornering margin).
2. **Missing EKF2 vision-sensor lever arm.** `EKF2_EV_POS_X/Y/Z` (the offset of the vision reference point from the vehicle's own IMU, in FRD body frame) was never set anywhere in this repo — confirmed via `grep`. The published `/fmu/in/vehicle_visual_odometry` represents the `d435i` model's OWN onboard IMU (co-located with the color sensor, per part 3's fix), physically offset from the flight controller's `base_link` IMU by the `CameraJoint` mount pose (`0.12, 0.03, 0.242` m forward/left/up, FLU). Without telling EKF2 about this, it silently assumed vision measurements originate at its own IMU — wrong by that lever arm — so any attitude change (pitch/roll) makes the offset sensor translate through space in a way the vehicle's true center does not, and EKF2 misattributes that as genuine motion. Landing/correction maneuvers involve more attitude change than steady cruise, so this would bite hardest exactly where the divergence was observed. **Fixed**: `set_localization_source.py` now also sets `EKF2_EV_POS_X=0.12, EKF2_EV_POS_Y=-0.03, EKF2_EV_POS_Z=-0.242` (FRD-converted) via a new float-param MAVLink path (`_set_float_param` — distinct from the existing int32-bitmask `_set_param`, confirmed necessary since these are genuine PX4 FLOAT params) whenever `--source vision` is selected.
3. **Visual feature starvation near the ground.** `vio_test.sdf`'s ground plane was a flat, uniform-gray 500×500 plane; the "fence" props are TALL (0.35-1.6m) and, confirmed by reading the world file's own geometry, positioned such that at low altitude near the landing point their tops exit the D435i's 42° vertical FOV — leaving only the textureless ground plane in view, zero KLT corners, forcing OpenVINS onto unconstrained IMU dead-reckoning at exactly the moment (final descent/touchdown) precision matters most. **Fixed**: added a 64-tile (8×8, 1m² each), alternating light/dark checkerboard of thin ground-level boxes covering the whole mission footprint and landing zone — same proven "colored box, not a texture/material asset" technique already used successfully for the fence props, zero missing-asset risk. Confirmed live afterward: OpenVINS tracked 42-44 features pre-arm post-fix, up from ~27 before.

**Retested twice on independent fresh stacks (full `docker compose down/up` each time) — BOTH fully automatic, zero manual intervention, zero divergence:**
- Run 1: climb → all 4 waypoints (zero unexpected resets) → landing → `has_low_throttle` fallback engaged once, disarmed immediately (force flag from part 6 worked) → `"Landed and disarmed — mission complete"`. Ground truth: `x=1.15, y=0.17, z≈-0.01` (resting), `landed: True` (PX4's OWN land detector agreed this time, not just the fallback), `freefall: False`.
- Run 2: identical shape — 4 waypoints, zero unexpected resets, landing, single fallback trigger, immediate disarm, `"mission complete"`. Ground truth: `x=1.18, y=0.39, z≈-0.01`, consistent with run 1.

Neither run showed anything resembling part 6's chaotic descent or climb-away — `vz` stayed smooth and bounded (~0.65-0.75 m/s) through both entire descents, matching the well-behaved pattern from the ORIGINAL successful parts 2/3 runs, not the diverging parts 6 runs.

**Honest caveat**: 2 clean runs is meaningfully stronger evidence than the 1-clean-then-2-diverged pattern that preceded it, but this session's own history (parts 2/3/5's clean runs, then part 6's 2-for-2 divergence with an UNCHANGED config) already proved SITL run-to-run variance is real here. Treat this as a strong, well-evidenced fix — not a mathematical guarantee — and watch for recurrence, especially before trusting it unattended on real hardware. The `land_disarm_max_timeout_s` safety backstop (part 6) remains in place regardless, so even a future recurrence would fail safe (stay armed, require manual intervention) rather than silently disarm wrong.

## Phase 3 Milestone B result, part 8 (2026-07-09, same day) — user-reported GUI bug fixed, README fully updated for public/replication use

**User ran `PX4_GZ_WORLD=vio_test make sim-gui` themselves to do their own
visual verification of the 3 flight tests, hit two real bugs immediately —
both fixed:**
1. **`PX4_GZ_WORLD=vio_test` silently had no effect** — `.env` hardcoded
   `PX4_GZ_WORLD=empty`, and because the Makefile does `include .env` +
   `export`, that plain assignment always wins over a same-named
   environment/command-line variable in GNU Make, regardless of the
   Makefile's own `PX4_GZ_WORLD ?= empty` default (which exists
   specifically to be overridden). Fixed by removing the line from `.env`
   entirely — it's a per-invocation runtime choice, not a build-time
   constant like the version pins around it, so it doesn't belong there.
2. **The drone was missing from the Gazebo GUI's 3D view/entity tree even
   though the world (checkerboard/props) rendered fine and the sim was
   flying correctly** — confirmed via `gz service .../scene/info` (returns
   the vehicle's full mesh/pose data) and `gz topic -e .../pose/info` (live
   correct pose) that this was a GUI-client rendering issue, not a
   simulation bug. Root cause candidate the user found and reported:
   PX4's `pxh>` shell, given no TTY (the default for `docker compose up -d`),
   spins retrying reads on closed/EOF stdin — confirmed via hundreds of
   repeated `[2Kpxh>` prompt-redraw lines in `docker logs px4-sim`. Fixed
   at the root with `tty: true` + `stdin_open: true` on `px4-sim` in
   `docker-compose.yml` (confirmed live: `pxh>` spam gone, `px4`'s own CPU
   usage dropped substantially). Caveat found during verification: the
   dominant CPU consumer post-fix is still Gazebo itself (`gz sim`, shown
   as `ruby` in `top` — that's `gz-tools`' actual CLI/runtime, not
   overhead) at real-time-factor well under 1.0 on this laptop — this is
   likely genuine load from the ~100 static prop/tile entities the part-7
   VIO fix added, not a bug, and means occasional GUI-sync races could
   still theoretically happen on this class of hardware even with the tty
   fix — and indeed one DID happen on the very next user run (see below).

**CORRECTION, later the same day — the `scene/info` service-call version of
`make gz-resync` does NOT work; rewritten to restart the GUI client
instead.** The user hit the invisible-drone state again after the tty fix
and ran the original `make gz-resync` — no effect. On reflection the
original mechanism was wrong: a `gz service` call's response goes to the
CALLER of the service (the CLI), not to the desynced GUI client, so it
teaches the GUI nothing. (The earlier belief that it "worked" came from a
run where the timing coincided with something else.) What actually works,
verified live on the user's stuck session: kill and relaunch only the
`gz sim -g` GUI-client process — it reconnects and downloads the complete
scene fresh; the sim server/PX4/any in-progress flight are untouched (the
GUI is a pure viewer). Two implementation details that matter: the relaunch
must re-export `GZ_SIM_RESOURCE_PATH` (else the fresh GUI can't resolve the
drone's `model://` mesh URIs — same invisible-drone symptom, different
cause) and `GZ_IP` — both are set inside PX4's launch-script session, NOT
in the container's top-level env that `docker exec` inherits (verified by
diffing `/proc/<pid>/environ` of the running GUI vs. the container env).
`make gz-resync` and README §4.3/issue 20 both rewritten accordingly.

**README.md substantially rewritten** to reflect this entire session's
work (parts 2-8) for a fresh reader trying to replicate/use this repo
without hitting any of the same issues — not just a changelog pointer.
Added/fixed: `PX4_GZ_WORLD=vio_test` requirement for `VIO_BACKEND=openvins`
surfaced prominently in §4.3/§8 (was previously undocumented in the main
walkthrough entirely); `make gz-resync`; the `.env`-override bug as its own
warning; `common_perception` added everywhere it was missing (§2, §4.4,
§13); the two new required `land_disarm_*` params (§7); VIO/camera/IMU
topics (§6); the d435i model swap (§8, §11); stale example values fixed to
match real `sim_params.yaml` (side_length_m 3.0 not 4.0, missions'
max_velocity_m_s 0.8 not the old 0.2 or a copy-pasted 1.0, survey area
5x5m/0.5m spacing not 8x6m/2m); §12 Known Issues grew from 16 to 23 entries
(the camera extrinsics bugs, EV_POS lever arm, IMU-excitation/velocity
fix, feature-starvation/checkerboard fix, the `.env` override bug, the GUI
fix, and the post-landing disarm-fallback mechanism, each in the same
"what broke — why — how fixed" style as the existing 16); Phase 3 marked
complete in the roadmap with an honest caveat about run-to-run variance
rather than an unqualified "done."

## Phase 3 Milestone B result, part 9 (2026-07-09, same day) — user-reported square-mission CRASH investigated; not reproduced directly, but a real, related process-lifecycle bug found and fixed; added a geofence as defense-in-depth

**User report**: ran `LOCALIZATION=vision VIO_BACKEND=openvins make mission
MISSION=square` themselves, watching in the Gazebo GUI, and the vehicle flew
out of the fenced/textured area at speed, kept going, and hit the ground —
a real crash ("totally destroyed"), not a log-only anomaly. Containers had
already been torn down by the time this was reported, so no logs from the
actual incident were recoverable.

**Reproduction attempts**: confirmed the user's exact command via
`AskUserQuestion`, then ran it live myself, twice, from fresh containers
(`docker compose --profile sim down` + `PX4_GZ_WORLD=vio_test docker
compose --profile sim up -d` each time — never a partial restart). **Both
runs completed cleanly** — all 4 waypoints reached accurately, landed, and
auto-disarmed via the part-6 fallback, zero warnings/errors during flight.
Confirmed the user did a fresh container restart between their flight-test
and the crashing mission too (asked directly), ruling out reused-container
staleness (a previously-documented gotcha) as the explanation. Given this
session's own part-7 caveat — "2 clean runs is strong evidence, not a
guarantee, this same config ran clean 3x then diverged 2x with nothing
changed" — the most likely explanation is genuine run-to-run VIO
divergence, stochastic rather than deterministic, that simply didn't
recur in these particular attempts.

**A real, separate, worse bug was found in the process, while cleaning up
between reproduction attempts**: a `square_mission` process from an
*earlier* test run was still alive many minutes after logging "Landed and
disarmed — mission complete." The OpenVINS instance sharing its launch —
now with no mission logic left to sanity-check it and no ground truth to
anchor it — ran its own position estimate away unbounded: `p_IinG` grew
from near-zero to `(18, 111, 28)` meters, `dist` (total path length) to
150 m, over a few minutes, accompanied by repeating `Propagator::
select_imu_readings(): Zero DT` warnings. Killing that stale process and
restarting fresh, `vehicle_local_position` on the NEXT boot came up at
`x=53, y=-103, z=-12` — PX4's own EKF2 had fused enough of the runaway
vision odometry before disarm to leave a genuinely corrupted estimate that
then blocked arming on the following flight. **Root cause**: `rclpy.
shutdown()`, called from `FlightState.DONE`, does not reliably unblock
`rclpy.spin()` in this stack — confirmed live, the process stays alive and
consumes CPU well past its own "mission complete" line — most likely
because `rmw_cyclonedds_cpp` (this repo's `RMW_IMPLEMENTATION`) leaves
non-daemon background threads a plain interpreter exit won't wait past.
Since the OS process never actually exits, `autonomy.launch.py` had no way
to know the flight was over either — there was no `on_exit` handler wired
to anything in the first place, so even a correctly-exiting node wouldn't
have torn down its sibling OpenVINS/bridge nodes.

**Fixed with two changes, together — took three iterations on the first
change to actually work, each verified live rather than assumed**:
1. `common_missions/launch/autonomy.launch.py` wraps the flight/mission
   `Node` action with `RegisterEventHandler(OnProcessExit(...,
   on_exit=Shutdown()))`, so the instant that process exits — success or
   failure — the whole launch (VIO, camera/IMU bridge, everything) tears
   down with it. This part worked as written the first time, but stayed
   silently untested until fix 2 below was right, since it only fires once
   the target process actually exits.
2. Getting the flight/mission node's OWN process to actually exit took
   three attempts:
   - **Attempt 1**: added `os._exit(0)` as the last line of
     `offboard_control_node.py`'s `main()` and `mission_base.py`'s
     `run_mission()`, after the existing `rclpy.shutdown()` in `finally`.
     **Did not work** — confirmed live, the process was still alive and
     consuming CPU minutes after "mission complete." Root cause: control
     never reached that line at all, because `rclpy.spin()` itself never
     returned — the bug was inside `spin()`, not after it.
   - **Attempt 2**: moved the exit call directly into `FlightState.DONE`
     in the control loop, as `rclpy.shutdown(); os._exit(0)`, reasoning
     that `DONE` runs unconditionally on every tick once entered so it
     shouldn't matter whether `spin()` ever returns. **Also did not
     work**, same symptom. Root cause: `rclpy.shutdown()` called
     synchronously from inside a timer callback running ON the executor's
     own spin thread appears to block/deadlock waiting on that same
     thread — so `os._exit(0)`, the very next line, never ran either.
   - **Attempt 3 (working)**: dropped the `rclpy.shutdown()` call from
     `FlightState.DONE` entirely, leaving just `os._exit(0)` — graceful
     rclpy teardown is moot anyway one line before a hard OS-level
     process kill. **Confirmed live**: the launch log showed
     `offboard_control_node-2: process has finished cleanly`, immediately
     followed by `launch: process[...] was required: shutting down
     launched system` (fix 1 firing correctly) and SIGINT sent to all
     three sibling processes. `ps aux` afterward showed zero leftover
     `offboard_control_node`/`run_subscribe_msckf`/
     `openvins_odometry_bridge`/`parameter_bridge`/`ros2 launch`
     processes — where previously three separate generations of them had
     been found stacked up concurrently in the same container. One minor
     cosmetic blemish, not chased further: `run_subscribe_msckf` (OpenVINS)
     exits via SIGSEGV rather than cleanly when SIGINT'd mid-computation —
     harmless, since it's being killed either way, but not a graceful
     shutdown on OpenVINS's own part.

**Also added, addressing the original report directly** (not just the
process-lifecycle side-finding): a geofence in `offboard_control_node.py`.
Rather than keep chasing a stochastic, not-directly-reproduced trigger,
this is defense-in-depth independent of root cause — bounds are
auto-derived from the current route's own bounding box (origin + every
queued waypoint, so it automatically follows whatever a mission's own
`side_length_m`/`area_length_m`+`area_width_m` describe, no mission-
specific code needed) plus new required params `geofence_margin_m`/
`geofence_height_margin_m` (§7 of README.md). A breach in any flying state
(TAKEOFF/WAYPOINTS/HOVER) aborts immediately to `LAND`, reusing the
existing estimate-independent disarm fallback from part 6. Verified live:
a clean hover flight-test and a clean square mission both completed
normally post-fix with zero false triggers (bounds sit well outside the
route but well inside `vio_test`'s fenced/textured area — fence spans
roughly [-1.5, 8]×[-1.5, 8] m, square mission's fenced bounding box with
margin is [-2, 5]×[-2, 5] m).

**Honest caveat**: the geofence and the process-exit fix are both real,
independently-valuable fixes, but neither is a confirmed root-cause fix
for the original crash report — it was never directly reproduced. Treat
the geofence as a safety net that bounds the BLAST RADIUS of a future
recurrence (an early forced landing instead of a runaway crash), not as
proof the underlying stochastic VIO-divergence issue (part 7's caveat) is
solved.

## Phase 3 Milestone B result, part 10 (2026-07-09, same day) — live camera preview (rqt_image_view) made part of the stack's own lifecycle, fixing a real sequencing race

**User workflow**: user manually ran a throwaway `docker run` for
`rqt_image_view` per mission (given as a one-off command in this same
session), and reported it working for a `square` mission but, on the next
`survey` mission, taking a long time with no live feed ever appearing.
Diagnosed live: `ros2 topic info --verbose` showed the topic existed
(created by the subscriber) but **Publisher count: 0**, and `ros2 node
list` showed nothing running except the viewer itself — the survey mission
had already finished and, thanks to part 9's `on_exit=Shutdown()` fix,
fully torn its whole pipeline down (camera bridge included) before the
manually-started viewer connected. Confirmed this is not RViz-vs-rqt
specific — any subscriber started after the mission's teardown loses the
same race, so switching viewers would not have helped.

**Fixed by making the viewer part of the stack's own lifecycle** rather
than a manually-timed per-mission command: added a third service,
`rqt-viewer`, to `docker-compose.gui.yml`, started automatically by `make
sim-gui` alongside `px4-sim`/`ros2-autonomy`. It subscribes to
`/camera/camera/color/image_raw` from the moment the stack comes up —
*before* any mission's camera bridge exists — so DDS discovery connects
the two lazily whenever a `VIO_BACKEND=openvins` mission's bridge actually
starts publishing, regardless of ordering after that point. Also baked
`ros-humble-rqt-image-view` into `Dockerfile.ros2_autonomy` (was
previously `apt-get install`ed at container-start time in the throwaway
command, costing ~30-60s on every single run).

One real implementation gotcha, not just a config nicety: `rqt-viewer`
reuses the `ros2-autonomy` image (same RMW, same network namespace via
`network_mode: host`) to avoid building a whole separate image, but that
image's `ENTRYPOINT` starts a `Micro-XRCE-DDS-Agent` — a second instance
bound to the same host UDP port `ros2-autonomy` already uses would either
fail to bind or fight over the same PX4 DDS stream. Overrode
`entrypoint: []` on the new service to skip it; `rqt-viewer` only needs
`rqt_image_view`, no DDS agent of its own.

Also fixed in the process: `make stop` previously only ran `docker compose
--profile sim down` against the base `docker-compose.yml` — a container
started via the `docker-compose.gui.yml` overlay (like this new one, or
Gazebo's GUI-mode `px4-sim`) would become an orphan `down` doesn't know
about unless invoked with the same `-f` files. `make stop` now always
passes both compose files plus `--remove-orphans`, so it fully cleans up
regardless of whether `make sim` or `make sim-gui` was used last.

**Verified fully live, not just config-reviewed**: rebuilt the
`ros2-autonomy` image (confirmed `ros-humble-rqt-image-view` installs
cleanly), brought up the real `make sim-gui` stack with a genuine X11
session available, confirmed `rqt-viewer`'s container stayed alive with no
X-connection error and its node was DDS-visible from `ros2-autonomy`
BEFORE any mission ran. Then ran a real `action:=hover
localization_source:=vision vio_backend:=openvins` flight and confirmed,
via `ros2 topic info --verbose` and `ros2 topic hz`, that the pre-started
viewer showed up as a live subscriber the moment `camera_imu_bridge`
started publishing, with frames actually flowing (~2 Hz). `make stop`
confirmed to tear down all three containers, `rqt-viewer` included.

## Known gotchas for whoever continues this
- **Restarting px4-sim alone wedges the DDS bridge.** If the px4-sim container
  is recreated/restarted while ros2-autonomy (and its Micro-XRCE-DDS-Agent)
  keeps running, the client/agent session re-establishes on paper (agent log
  shows "session re-established", datawriters created) but no data flows and
  `uxrce_dds_client status` inside PX4 reports "Running, disconnected" with
  timesync never converging. Restarting the agent process alone did NOT fix
  it. Full `docker compose --profile sim down && up` does. Rule of thumb:
  always restart the whole sim stack together. (Candidate future fix: tie
  container lifecycles together or add an agent healthcheck/restart policy.)
- **PX4 arming preflight on x500_depth SITL needed two param overrides**, both
  applied via the SITL-only airframe hook
  `docker/px4_sitl_overrides/4002_gz_x500_depth.post` (never reaches real HW):
  1. `CBRK_SUPPLY_CHK 894281` — model has no battery/power sim; battery+power
     checks would fail forever.
  2. `NAV_DLL_ACT 0` — the x500 airframe sets 2, which makes a GCS connection
     mandatory for arming (`rcAndDataLinkCheck.cpp`); we fly offboard with no
     QGroundControl. 0 is PX4's code default. Real-HW failsafe policy is a
     deliberate Phase 4 decision.
  Debugging tip that cracked it: `docker exec -w /PX4-Autopilot/build/px4_sitl_default/rootfs
  px4-sim /PX4-Autopilot/build/px4_sitl_default/bin/px4-listener health_report`
  gives `arming_check_error_flags` as a bitmask; decode bits against
  `health_component_t` in `build/px4_sitl_default/events/common_with_enums.json`.
  The `px4-*` client binaries (px4-listener, px4-commander, px4-param) work
  from `docker exec` as long as cwd is the SITL rootfs — far more reliable
  than scraping the pxh console log.
- **"Preflight Fail: ekf2 missing data" at boot is a transient**, not the real
  blocker — EKF2 needs ~30-60 s after boot before `pre_flight_checks_pass`
  goes true. Wait for it before arming (poll `vehicle_status_v1`).
- The topics `/fmu/out/vehicle_status` and `/fmu/out/vehicle_local_position`
  from PX4's docs/examples are published as `..._v1` in this PX4 version
  (message-versioning, `MESSAGE_VERSION` field in the .msg). `common_control`
  already subscribes to the `_v1` names.
- `DONT_RUN=1` does NOT stop the new Gazebo (`gz_x500_depth`) target from
  launching — only Gazebo Classic/jMAVSim honor it. Build-only step must use
  `make px4_sitl_default` (see `Dockerfile.px4_sim`).
- `ros:humble-ros-base` does not ship `rmw_cyclonedds_cpp` — must be apt
  installed explicitly (see `Dockerfile.ros2_autonomy`, appended *after* the
  expensive `colcon build` layer so editing it doesn't bust that cache).
- The Micro-XRCE-DDS-Agent is NOT started automatically by `make sim` yet —
  currently started manually inside `ros2-autonomy` via
  `docker exec -d ros2-autonomy bash -c "MicroXRCEAgent udp4 -p 8888 ..."`.
  This needs to become part of a proper launch file / entrypoint in Phase 1
  rather than a manual step.
- `ros2 topic list` / any ROS 2 CLI usage inside `ros2-autonomy` currently
  needs `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` exported manually in some
  shells even though it's set at the container `environment:` level in
  compose — worth double-checking this is actually inherited correctly by
  `docker exec` sessions before Phase 1, or just export it explicitly in
  whatever entrypoint script gets written.
- Container image tags are hardcoded to version numbers
  (`quad/px4-sim:v1.17.0`, `quad/ros2-autonomy:humble`) — if `.env`'s
  `PX4_VERSION` changes, the `image:` line in `docker-compose.yml` needs a
  matching manual update (compose doesn't template that field from `.env`).

## Phase 3 Milestone B result, part 11 (2026-07-09, same day) — `survey_mission` removed from the repo (temporarily — not a retraction of the pattern)

**User request**: after confirming the rqt-viewer sequencing fix (part 10)
was working, explicitly asked to remove `survey_mission` from the repo —
"I do not want any survey mission for now... I will design different
mission later with different world" — with an explicit instruction not to
touch or break anything else working (`square_mission`, `common_control`,
`flight-test`, etc.).

**Not a bug fix — a scope decision.** `survey_mission` itself was working
(flew successfully in earlier testing, per parts 2/7 above); the removal
is about the user not wanting a lawnmower-pattern mission sized for the
current `vio_test` world footprint cluttering the repo while they design a
differently-shaped mission against a differently-shaped world later. The
`MissionBase` pattern this relied on (declare geometry params, return a
waypoint list, ~25 lines) is untouched and still the intended path for
whatever comes next.

**Removed, surgically — nothing outside `survey_mission`'s own footprint
touched**:
- `ros2_ws/src/common_missions/common_missions/survey_mission.py` — deleted.
- `common_missions/setup.py` — removed its `console_scripts` entry point;
  description string updated (was "square, survey").
- `common_missions/launch/mission.launch.py` and `.../autonomy.launch.py`
  — `MISSIONS` tuples now `('square',)`; docstring examples using
  `mission:=survey` fixed to `mission:=square`.
- `sim_bringup/launch/sim.launch.py` — same docstring/description fixes.
- `sim_bringup/config/sim_params.yaml` and `hw_bringup/config/
  hw_params.yaml` — `survey_mission:` YAML sections deleted outright
  (nothing else in either file references them).
- `README.md` — every currently-actionable reference (command examples,
  §5 command table, §7 parameter tables/YAML example, §9 launch-file
  argument reference, §13 repo layout, §14 roadmap) updated to reflect
  `square` as the only mission today, each with a one-line note that
  `survey` existed, worked, and was removed by choice, not because it
  broke — not silently deleted from the record. Historical narrative
  (§12 Known Issues, e.g. issue 17's IMU-excitation fix) left untouched —
  those describe what was true when the bug was found and fixed, still
  accurate as history regardless of what missions exist today.
- `resource/phase3-gps-denied-localization-source.md` — deliberately left
  untouched (a historical Milestone A testing checklist, not live
  instructions — editing it would be exactly the kind of unnecessary
  churn the user asked to avoid).

**Not touched, confirmed by scope of the diff**: `common_control` (the
entire offboard/geofence/disarm-fallback state machine), `square_mission.py`,
`mission_base.py`, `docker-compose*.yml`, any Dockerfile, and
`DEVELOPMENT_STATUS.md`'s own historical entries (parts 1-10 above) —
survey mentions throughout stand as accurate history of what was tested at
the time, not edited to pretend it never existed.

## Phase 3, part 12 (2026-07-10) — RViz2 live viz added; VIO reliability investigation (3 fixed, 1 root-caused as a SITL performance issue — see part 13)

**Part A — RViz2 added, same session, before the reliability work below**:
`common_perception/state_tf_publisher.py` (new node) broadcasts TF
(`odom`->`base_link`) + `/drone/path` from `/fmu/out/vehicle_odometry`,
paired with RViz2 via `common_perception/launch/viz.launch.py` and a saved
`config/quad.rviz`. New `rviz2` service in `docker-compose.gui.yml`,
started by `make sim-gui` alongside the existing `rqt-viewer`, mission-
lifecycle-independent (same "up from the start" pattern as `rqt-viewer`).
Verified live: TF/path populate from PX4 boot (no mission needed), RViz
renders with real GPU OpenGL, survived a full hover flight-test. One real
bug found+fixed: `px4_msgs` fields are numpy `float32`, `geometry_msgs`
setters reject them — needed explicit `float()` casts.

**Part B — user reported real square-mission (VIO_BACKEND=openvins)
reliability problems**: high drift some runs, hit the `vio_test.sdf`
fence on others, not reproducible run-to-run. Also reported RViz's camera
image pane not updating properly. User supplied `resource/
Vio_Drift_analysis.txt` (an external analysis doc) for review.

Reviewed against actual code/config (not taken at face value) — mostly
correct, one arithmetic error found (image bandwidth math used 1
byte/pixel instead of RGB8's 3; real number ~83MB/s not ~27.6MB/s, if
anything strengthens the doc's own conclusion). User chose "quick wins
first, test, then stereo" as the plan (not "do everything blind").

**Three real bugs fixed, individually verified**:
1. **EKF2 vision-noise floor too permissive.** PX4's `EKF2_EV_NOISE_MD=0`
   default trusts OpenVINS's own raw per-message covariance above a
   parameter FLOOR (`EKF2_EVP_NOISE`/`EKF2_EVV_NOISE`, PX4 default
   0.1m/0.1m/s) — `openvins_odometry_bridge.py` forwards that raw
   covariance untouched, so EKF2's trust could swing with however
   confident OpenVINS felt on a given frame. Raised the floor to
   0.3m/0.15m/s in `set_localization_source.py` (`vision` switch only).
   **First attempt was `EKF2_EV_NOISE_MD=1`** (ignore message entirely,
   fixed trust) — confirmed live to correlate with arming failures (see
   Part C) — reverted to the floor-only fix, which does NOT touch
   `EKF2_EV_NOISE_MD` (stays at PX4 default 0).
2. **`vio_test.sdf` ground tiles were a strict 2-color checkerboard** — a
   textbook aliasing trap for a corner tracker (every tile corner locally
   identical to every other). Re-colored the same 64 tiles (same
   positions, same "plain box, no texture asset" technique) with a fixed
   deterministic hash across an 8-color palette — no periodic pattern,
   still fully reproducible. **`vio_test.sdf` is baked into the `px4-sim`
   image at build time (`Dockerfile.px4_sim` COPYs it) — required a full
   `make build`/image rebuild, not just `make build-ws`, to take effect.**
   Verified: rebuilt image, confirmed new colors present in a fresh
   container.
3. **RViz2's `Image` display was a real, separate, UI-only bug** — a
   second subscriber to the same ~83MB/s raw camera stream `rqt-viewer`
   already covers, plus RViz's `Image` plugin uploads a GPU texture on
   the same thread as the 3D view. Dropped from `quad.rviz` entirely —
   `rviz2` is TF/path only now, `rqt-viewer` remains the camera viewer.
   Never touched OpenVINS itself (subscribes to the DDS topic directly),
   so this was never a flight-safety issue, just a UI one.

**Part C — a FOURTH issue found WHILE verifying the above, root-caused the
same day (see part 13 below)**: across 4 clean, fully-restarted (`docker
compose down`/`up` each time, never a partial restart) flight attempts on
the FINAL (floor-only, safe) config, 3 armed/flew the full `square`
mission/landed/disarmed normally, 1 never armed within 260s, stuck on
PX4's own "Preflight Fail: ekf2 missing data" / "waiting for estimator to
initialize" (`EstimatorCheck.cpp`) — the SAME symptom seen (and initially
misattributed solely to `EKF2_EV_NOISE_MD=1`) during the fix-1 regression
above, proving that param wasn't uniquely responsible either. **Do not
assume the 3 fixes above alone solved the user's original complaint —
they are real, individually-verified fixes for real bugs, but part 13's
RTF finding is plausibly a bigger share of the original flight-to-flight
inconsistency; re-test multiple times, ideally on an idle host, before
trusting reliability.**

Stereo VIO (real D435i left/right IR pair — Gazebo can simulate this fine,
no real IR-speckle-projector physics needed, just two cameras at a known
baseline) remains the user-approved next step after reliability work.
Would inherit part 13's RTF-dependent arming gap unchanged (not
mode-specific) — worth testing on a healthier-RTF host alongside/before
adding it, not strictly a hard blocker.

## Phase 3, part 13 (2026-07-10, later same day) — Part C ROOT-CAUSED: SITL real-time-factor collapse, not a code bug

User submitted a second analysis doc proposing wall-clock-vs-simulation
ROS clock mismatch (`use_sim_time`) as the cause of Part C, with a
concrete 2-step fix (set `use_sim_time:=true` everywhere; use OpenVINS's
own message timestamp in `openvins_odometry_bridge.py` instead of
`self.get_clock().now()`). Told to "go with your recommendation" after
review.

**Reviewed and REJECTED before implementing anything** — verified against
actual code/stack behavior, not taken at face value:
- `loopback_odometry_bridge.py` (Milestone A, reliable throughout this
  project's history) stamps `VehicleOdometry` with the EXACT same
  `self.get_clock().now()` wall-clock pattern `openvins_odometry_bridge.py`
  uses. If wall-clock stamping broke PX4/EKF2 fusion this badly, Milestone
  A would already be broken too — it isn't.
- `ros_gz_bridge.yaml` bridges no `/clock` topic to ROS 2 at all — setting
  `use_sim_time:=true` as proposed, without first adding that bridge,
  would starve every node's ROS clock of a source. Real risk of making
  things WORSE, confirmed by checking the actual bridge config before
  running anything.
- Checked PX4's own `EstimatorCheck.cpp` (the exact check that fires):
  `missing_data` is set purely by whether PX4's OWN INTERNAL
  `estimator_selector_status`/`estimator_status` uORB topics come back —
  nothing to do with an external vision message's timestamp domain. A
  genuine timestamp-buffer-rejection issue would show a DIFFERENT,
  innovation/consistency-related PX4 log message, never observed.

Given user said "go with your recommendation" (adopt the message-
timestamp fix, skip `use_sim_time`, go instrument a live stuck run
instead) — went to implement the timestamp fix, but verification caught a
NEW problem with adopting even that: OpenVINS's own message header stamp
is in a DIFFERENT domain (Gazebo-simulation-derived, small values) than
`self.get_clock().now()` (wall-clock, huge epoch values) — using the
message's stamp for ONLY `timestamp_sample` while leaving `timestamp` on
wall-clock would create a NEW internal inconsistency between the two
fields that doesn't currently exist. Did NOT implement it — correctly
walked this back to the user rather than applying a plausible-looking fix
that hadn't actually been proven safe.

**Actual root cause, traced through OpenVINS's own source
(`ov_msckf/src/ros/ROS2Visualizer.cpp`, NOT this project's code) and
confirmed live**: `visualize_odometry()` unconditionally returns without
publishing ANYTHING until `(timestamp - _app->initialized_time()) >= 1` —
one full second of OpenVINS's own internal, Gazebo-simulation-derived
time since init. Confirmed live during a reproduced stuck attempt:
- `gz topic -e -t /world/vio_test/stats` measured `real_time_factor:
  0.023` (~1/44th of real time) — `gz sim` alone was consuming 448% CPU
  on this 8-core host, load average >5 (after hours of continuous
  `docker compose` restarts/exec calls this session).
- `ros2 topic hz /fmu/in/vehicle_visual_odometry` showed ZERO messages
  across a full 250-second wall-clock window during the stuck run.
- Ruled out a DDS discovery race specifically: `ros2 topic info
  /ov_msckf/odomimu --verbose` during a live standalone run showed
  OpenVINS's publisher and `openvins_odometry_bridge`'s subscriber BOTH
  correctly discovered with fully compatible QoS (RELIABLE/VOLATILE both
  sides) — the block is entirely inside OpenVINS's own gate, not a
  discovery/connection problem.
- OpenVINS's own debug log showed it healthy the whole time (continuous
  ZUPT updates, one confirmed "successful initialization" line) — nothing
  crashed or hung; it was correctly waiting exactly as its own source
  code says to.

At `real_time_factor: 0.023`, OpenVINS's 1-second (sim-time) gate alone
needs ~44 REAL seconds just to START publishing, and can take arbitrarily
longer if RTF degrades further — easily exceeding any reasonable
arm-retry patience, entirely explaining Part C without any code defect
anywhere in this project, PX4, or OpenVINS.

**100% SITL-specific, cannot recur on Phase 4 real hardware** — confirmed
PX4 SITL runs in Gazebo's lockstep mode (subscribes to
`/world/<world>/clock`, per `px4-rc.gzsim`); real hardware has no
simulated clock at all, so this exact mechanism structurally cannot occur
there. Given the ORIGINAL report was inconsistent flight-to-flight
behavior, RTF variance under host CPU load (worse when other things are
running, exactly this session's own condition) plausibly explains a real
share of that directly, independent of Phase 3/12's 3 fixes.

**No code fix applied — this needs mitigation, not a patch**, since the
gating mechanism is inside OpenVINS's own vendored source, not this
project's code to change (§11 README, no upstream forks). Options for
next session/user testing, roughly priority order:
1. Reduce simulation computational load: fewer/lighter `vio_test.sdf`
   props, lower camera resolution/framerate (would also serve part 12's
   RViz-bandwidth fix — but needs re-deriving
   `kalibr_imucam_chain.yaml`'s intrinsics for the new resolution, a real
   recalibration, not just a config edit — get explicit buy-in before
   doing this, it's bigger/riskier than anything else on this list), or
   reduce OpenVINS's `track_frequency`/`num_pts` if still bottlenecked.
2. Test on a genuinely idle host — this session's 0.023 RTF was measured
   after hours of continuous restarts; a fresh baseline (reboot, nothing
   else running) may be substantially better and worth establishing
   before concluding anything is still broken.
3. Don't assume "stuck" from a short timeout in future test tooling —
   check `gz topic -e -t /world/<world>/stats`'s `real_time_factor`
   directly before concluding something is broken.

Full write-up, evidence, and same options: README §12 issue 29.

## Phase 3, part 14 (2026-07-10, second half — a VERY long session) — 30° camera tilt implemented; FIVE deeper root causes found and fixed along the way; first tilted flight took off with in-flight VIO but DIVERGED 84m (open); read this before ANY further VIO work

User accepted the tilt plan (their Gemini-authored proposal, reviewed and
partially corrected — see below), with acceptance criteria: cm-level drift
over 5 clean runs, fallback mechanism only AFTER accuracy is proven.
Expectation was set honestly first: mono VIO typically drifts 0.5-1% of
distance (10-20cm over this mission); single-digit cm is stereo territory.

**Corrections made to the user's Gemini tilt proposal (all verified against
code/hardware reality, not vibes)**:
1. Its `T_cam_imu` rotation update is WRONG for this rig: the d435i's IMU
   is inside the camera and tilts WITH it (same rigid link in
   `d435i/model.sdf`, same as physical D435i), so camera<->IMU relative
   transform is mount-invariant — `kalibr_imucam_chain.yaml` must NOT
   change. Applying the proposal's matrix would have deliberately
   re-created the Milestone B extrinsics failure class. What actually
   changes is vehicle-body<->camera-IMU, handled in
   `openvins_odometry_bridge.py` (new `mount_pitch_deg` param,
   compensation `q_vehicle = q_imu * conj(q_mount)`, `w_body =
   rotate(q_mount, w_imu)`, velocity path UNCHANGED — validated
   numerically 5 ways BEFORE flight AND validated offline AFTER the crash
   below against OpenVINS's real parked init attitude: raw decodes to
   exactly 30° pitch, compensated to exactly (0,0), flipped-direction
   would give 60° — the compensation is CORRECT and NOT the crash cause).
2. Its FOV table used the D435i depth/IR FOV (86x57°); this stack's mono
   pipeline uses the COLOR camera (69x42°) — with 42° vFOV, 30° tilt puts
   the FOV top edge 9° below horizon (not 1.5°): fence props mostly leave
   view at cruise, ground texture becomes almost the sole feature source.
3. Mount pose lives in `x500_d435i_depth/model.sdf` (include + CameraJoint
   pose, `.12 .03 .242 0 0.523599 0`), not `d435i/model.sdf`.

**Five deeper problems found by actually flying/probing, in causal order
(each was REQUIRED to find the next — none were visible until the previous
was fixed)**:

1. **RTF collapse root cause found — headless `px4-sim` had NO GPU access.**
   `docker-compose.yml`'s base service lacked `/dev/dri` (only the GUI
   overlay mapped it), so BOTH simulated cameras software-rendered on CPU:
   RTF 0.015, `gz sim` >400% CPU on an idle host. Adding `devices:
   /dev/dri` to the base service: **RTF 0.015 → 1.00** (44x), CPU 416% →
   178%. This retroactively supersedes part 13's "test on an idle host"
   advice — the host was never the problem; the missing GPU was. (Gemini
   checkpoint accepted: this makes /dev/dri mandatory for container start
   — a portability caveat on GPU-less machines, documented in README.)

2. **Tilted camera at ground level was feature-starved → OpenVINS could
   not initialize, SILENTLY.** Diagnosed with real captured frames + cv2
   FAST at the estimator's own threshold (30): 5 corners (needs 15+).
   TWO stacked causes: (a) the vehicle's own sun shadow covers the entire
   near-field ground view with soft penumbra (shadows now OFF in
   vio_test.sdf — measured, documented in the world file); (b) solid-color
   tiles fundamentally only give corners at 3-color junctions — at 0.3m
   range even 0.25m tiles fill hundreds of px each. REAL fix: a proper
   multi-scale image texture — `textures/vio_ground_mosaic.png` (seeded
   reproducible generator checked in next to it), ONE 8x8m textured pad
   replacing 128 tile entities. Result: 260 FAST corners at threshold 30,
   OpenVINS initializes on the ground reliably. The "silent" part:
   InertialInitializer returns false with NO log line when its feature DB
   is empty (found via DEBUG verbosity — which the ov launch OVERRIDES
   from its own launch arg, YAML `verbosity` is ignored — plus manual
   FAST on captured frames).

3. **OpenVINS structurally publishes NOTHING while parked** —
   `initialized()` requires `timelastupdate != -1`, set only by a real
   feature update, but every parked frame takes the pre-takeoff ZUPT
   early-return. Verified in VioManager.cpp/.h and live across boots
   (poseimu AND odomimu both silent post-init while parked; one earlier
   "63Hz while parked" observation was a spawn-settle bounce latching one
   real update). CONSEQUENCES: (a) the `wait_for_vision` PRE-ARM gate
   built earlier this session is a structural DEADLOCK (vision waits for
   motion, motion waits for arming, arming waits for gate) — REMOVED from
   `sim.launch.py` (the node is kept for the future in-flight watchdog);
   (b) this, not RTF alone, is the deep mechanism behind part 13/issue
   29's arming flakiness.

4. **At RTF 1.0 the no-aiding window became real: PX4 refused takeoff.**
   With GPS off and vision absent-until-motion, EKF2 dead-reckons; its
   validity window (`EKF2_NOAID_TOUT`, default 5s) used to last ~4 wall
   minutes at RTF 0.02 — at RTF 1.0 it's a real 5s, arming+takeoff didn't
   fit: `mc_pos_control: invalid setpoints` → `Failsafe: blind land`,
   asymmetric near-idle motors, vehicle NEVER physically lifted (Gazebo
   truth static all run) while the est z drifted to the geofence ceiling
   — the "armed and climbing" log was a lie both times it appeared today.
   FIX: `set_localization_source.py` now also sets `EKF2_NOAID_TOUT` to
   its PX4 max, 10s, for vision mode (5s default restored for gps mode).
   Also fixed en route: launch fail paths now `Shutdown()` instead of
   returning None (a gate timeout had orphaned a bridge — part-9 leftover
   -process class via launch sequencing); `timeout docker exec` kills the
   CLIENT not the in-container launch — always `pkill` inside or let
   missions exit by themselves.

5. **First REAL tilted flight (all four fixes in): armed, physically took
   off, VIO engaged in flight (281 update printouts — first time ever on
   the tilted rig) — then DIVERGED: estimate flew the commanded square
   "successfully" while the physical vehicle shot ~84m away and tumbled**
   (truth (2.0, 84.5), rpy ~(-1.2, -0.65, -1.2); est ended (-15, 36)).
   This is the user's ORIGINAL crash signature (part 9), now on the
   tilted rig. Geofence never fired because the ESTIMATE tracked the
   commanded path (classic diverged-VIO feedback loop). The bridge mount
   compensation is EXONERATED (validated offline against the crash-run's
   own init data — see correction 1 above). OPEN — prime remaining
   suspects for next session: (a) mono-VIO divergence under flight
   dynamics, possibly aggravated by the ~13Hz EFFECTIVE camera rate
   measured (30Hz configured — the old Haswell iGPU can't render both
   cameras at full rate even though physics holds RTF 1.0); (b) online
   intrinsics/timeoffset calibration (`calib_cam_intrinsics/timeoffset:
   true`) running away under aggressive motion at low frame rate (this
   project has prior form here — the extrinsics saga); (c) EV frame
   alignment interaction in EKF2. Next diagnostic: fly with the GUI on
   and the user watching, records of /ov_msckf/odomimu vs truth, and
   consider pinning online calib off + testing camera rate first.

**Net state of the tilt milestone: NOT flight-ready.** Ground init:
verified working. Takeoff + in-flight VIO engagement: verified working.
In-flight estimate quality: diverges — 1 crash in 1 real flight. The
5-run cm-accuracy acceptance testing CANNOT START until the divergence is
resolved. All infrastructure for it is ready (`make drift-report` prints
Gazebo truth + PX4 estimate; procedure: run before mission and after
landing, compare each source against its own baseline).

## Phase 3, part 15 (2026-07-13) — world+mission recentered on the spawn origin; THE DIVERGENCE IS NOT THE TILT — reproduced on master's forward camera at RTF 1.0

**User tested master (slow RTF) and reported**: hover + square both drift,
sometimes hitting the fence while landing / exiting the boundary; rqt image
sometimes "not updating". Root observations:
- The drift is the KNOWN stochastic mono-VIO instability (part 7 caveat) —
  master never had it fixed; the deep fixes are parked on dev/camera-tilt.
- "rqt not updating" at master's RTF ~0.026 is arithmetic, not a bug: the
  camera renders 30Hz in SIM time = <1 frame/s wall. Real fix is the
  /dev/dri pair on the branch.
- REAL DESIGN FLAW fixed this session (user-requested): the vehicle spawns
  at the world origin but the fence/tiles were arranged around the OLD
  first-quadrant mission footprint — landing point only 1.5m from two
  fence lines. Fence translated to ±4.75m, tiles ±4m (pure translation,
  commit 4f9e428), square_mission now flies corners ±side/2 around the
  origin with an explicit return-to-center waypoint (5 waypoints now, was
  4 — update any log-greps expecting 4). Landing clearance 1.5m → 4.75m.
  Geofence auto-follows (origin+waypoints bounding box). Verified live in
  GPS mode at RTF 1.0 (test-only GPU override, NOT committed): 5 waypoints,
  clean flight, landed 0.8m from center.

**BREAKTHROUGH while verifying — the part-14 84m divergence REPRODUCED ON
MASTER'S CONFIG** (forward camera, NO tilt, shadows on, solid tiles) the
moment it ran at RTF 1.0: vision square armed, OpenVINS initialized and
updated in flight (209 updates), then the estimate diverged within ~17s;
geofence aborted at estimated (-1.4, 3.8); vehicle physically ended 14.6m
out (landed upright — the centered fence + geofence prevented a prop
strike, but an estimate-based fence cannot prevent physical escape when
the estimate itself is wrong, documented limitation). **Conclusion: the
tilt, mosaic texture, shadows-off, and mount compensation are ALL
exonerated — the divergence correlates with RTF 1.0 itself.** Sharpest
suspect: at RTF 1.0 the iGPU delivers only ~13Hz effective camera rate
(measured), vs slow-RTF runs where the CPU renderer keeps the full 30Hz in
sim time — mono VIO gets 2.3x fewer frames per sim-second of motion at
real time. Second suspect: online intrinsics/timeoffset calibration under
real-time dynamics. Next investigation (branch): cap flight speed way down
at RTF 1.0 (fewer pixels/frame of motion ~ compensates low frame rate), or
reduce camera resolution so the iGPU can hold 30Hz (needs kalibr intrinsics
re-derivation), or freeze online calib. NOTE the divergence is therefore
ALSO the reason /dev/dri + NOAID_TOUT must NOT be backported to master yet
— confirmed empirically this session, not just cautious guessing.

Master remains the slow-RTF configuration the user's missions actually
complete on. Both master commits (fd344cd, 4f9e428) local; user pushes.

**SUPERSEDED BY PART 16 BELOW (2026-07-13, later same day): `/dev/dri`
WAS backported to master after all** — turned out the same RTF collapse
also broke GPS-mode reliability (not just vision-mode), so the "master
stays slow-RTF" conclusion above didn't hold. `EKF2_NOAID_TOUT` and the
rest of the vision-specific tilt-branch changes are still NOT backported —
only `/dev/dri` moved, and only after being re-verified safe for GPS mode
specifically. Read part 16 before assuming master's RTF configuration.

## Phase 3, part 16 (2026-07-13, later same day) — user tested master, reported GPS drift + incomplete square; root-caused as the SAME RTF/GPU gap hitting GPS mode too, plus a separate path-visualization bug; both fixed

**User report**: `make flight-test` took off/hovered/landed nicely but
"drifted during hover" with GPS. `make mission MISSION=square` "drifted a
lot during landing" and "did not complete the whole square path" —
attached an RViz screenshot of the `/drone/path` showing two tangled,
self-intersecting loops joined by a near-straight line, not a square.

**Investigation, not assumption**: checked live PX4 params on the running
container (`px4-param show`) and found `EKF2_GPS_CTRL=0` — the tested
flight was actually `LOCALIZATION=vision`, not GPS, and the vehicle was
physically tipped over (truth roll -1.15 rad) from the SAME already-open
part-14/15 in-flight VIO divergence. Restarted clean, then re-tested a
TRUE GPS-only hover and immediately reproduced a real, separate bug:

```
Armed and offboard — climbing to takeoff height
GEOFENCE BREACH: position (1.4, 2.1, -0.0) is outside x=[-2.0,2.0]...
Switching to land mode          <- 0.1s after arming
```

The very first position sample after arming was already garbage — on a
GPS-only flight, no camera/VIO involved at all. Root cause: **master's
`docker-compose.yml` never had `/dev/dri`** (only the GUI overlay and
part-14's dev/camera-tilt branch did) — confirmed RTF ~0.02-0.03 even
though nothing was using the camera. This README's own "PX4's EKF2 needs
~30-60s after `make sim`" guidance is REAL-time advice that silently
assumes RTF≈1; at RTF 0.02, 30-60 real seconds buys EKF2 only ~1-2
*simulated* seconds — nowhere near converged — while PX4's arm-gate is
looser than "fully converged," so the mission arms anyway onto a garbage
first estimate. Confirmed by direct A/B: added `/dev/dri` to
`docker-compose.yml` (test override first, then made permanent), waited a
REAL ~35s this time (meaningful now that RTF≈1), and: (1) GPS hover — 2m
climb, 5s hover, clean land, zero geofence trips; (2) GPS `square` — all 5
waypoints reached in exact order, landed ~2m from center (well inside the
4.75m fence clearance from part 15). Two consecutive flights back to back,
both clean.

**Separately, the screenshot's "tangled path" was ALSO real, but a
visualization bug, not a flight bug**: `state_tf_publisher` never resets
its `/drone/path` buffer between flights (by design — §4.3, "up from the
start", survives across missions) — so an EARLIER flight (from before the
crash) and the vision flight that crashed had their paths drawn as ONE
continuous connected line (last pose of flight A -> first pose of flight
B), which is exactly what a self-intersecting "didn't complete the
square" shape would look like regardless of what either flight actually
did. Fixed: `state_tf_publisher` now subscribes to `vehicle_status_v1`
and clears the path buffer on every disarmed->armed transition. Verified
live across the two clean flights above: reset log fired exactly twice,
once per arm.

**Net**: `/dev/dri` backported to the BASE `docker-compose.yml` (was only
in the GUI overlay + the tilt branch) — this is NOT the same call as
backporting the tilt branch's `EKF2_NOAID_TOUT`/vision changes, which
remain OFF master; `/dev/dri` alone is safe and now proven necessary for
GPS-mode reliability too, independent of the still-open vision-mode
divergence question (part 15). Both fixes committed together since they
were diagnosed together, but are logically independent:
1. `docker-compose.yml`: `/dev/dri` added to `px4-sim`'s base service.
2. `state_tf_publisher.py`: path buffer clears on arm.

Vision-mode (`VIO_BACKEND=openvins`) in-flight divergence remains OPEN —
unaffected by either fix here; still needs part-14/15's investigation
(camera frame rate at RTF 1.0, online calibration). GPS-mode should now be
reliable on master; re-test recommended before assuming otherwise.

## Phase 3, part 17 (2026-07-13, later same day) — camera-jitter fix tested (didn't clearly help accuracy); LAND-state geofence gap found and fixed (real safety win)

User asked to work on VIO next. Two sub-investigations, same session:

**A. Camera frame-timing jitter — fixed, but didn't clearly fix accuracy.**
Measured live: color camera at 1280x720 delivered only ~19-21Hz average
(configured 30Hz) with jitter up to 380ms gaps, even at a genuine RTF 1.0
— a rendering throughput limit on this iGPU, not a lockstep/RTF problem.
Traced `track_frequency` in OpenVINS's own source
(`ROS2Visualizer.cpp::callback_monocular`) and found it's a max-rate
CEILING that drops frames arriving too fast, NOT an assumed/expected
rate — my initial theory that the 30-vs-19Hz mismatch itself mattered was
WRONG, corrected before acting on it. Fix: depth camera throttled 30->5Hz
(unused stream, free win, minor ~8% color-rate improvement) and — the
real fix — color resolution 1280x720 -> 640x360 (`d435i/model.sdf`).
Caught and fixed my own arithmetic mistake before it shipped: first
attempt used 640x480, which silently changes aspect 16:9->4:3 and
distorts vertical FOV; 640x360 is the correct exact half-scale.
Intrinsics in `kalibr_imucam_chain.yaml` re-derived accordingly (fx/fy
465.7, cx/cy 320/180). Result: color camera now a rock-steady 30.24Hz,
std dev 2.3ms (was ~37ms), max gap 43ms (was 380ms) — a real, clean,
verified win on the jitter itself.

**Accuracy result was NOT a clear improvement, and may be a different
failure mode**: 6 total vision-mode `square` mission flights (3 before
this fix, 3 after, all headless/RTF1.0/settled). Pre-fix: 3/3 completed
all 5 waypoints, 1.7-5.8m drift at landing, no aborts. Post-fix: 1/3
completed (~2.25m drift), 2/3 aborted SAFELY during the initial climb —
geofence caught an early, large horizontal estimate divergence before
the vehicle (per Gazebo ground truth) had really gone anywhere. Zero
crashes either way (a real improvement over part 14's 84m tumble,
attributable to the settle-time/`/dev/dri` work, not this resolution
change specifically). Told the user honestly: small sample size, doesn't
prove regression, but doesn't support "problem solved" either — this
remains genuinely open mono-VIO territory.

**B. User reported repeated crashes testing vision mode**: run 1 drifted
(known, above); run 2 "hit the wall boundary during landing"; run 3
"landed far and tilted, hit ground." Investigated by reading
`offboard_control_node.py`'s actual state machine rather than guessing —
found `_check_geofence()` was called from `TAKEOFF`/`WAYPOINTS`/`HOVER`
but **never from `FlightState.LAND`**. Once a mission reaches "all
waypoints reached — landing," PX4's own `AUTO_LAND` controller takes over
descent with ZERO further oversight from this project's code — and this
project already has a documented prior incident of `AUTO_LAND` flying
away when its estimate diverges during descent (the extrinsics saga).
This directly and precisely explains run 2. Confirmed with the user this
was the right first fix before touching anything else.

Two-part fix, user-directed (they specifically also wanted the geofence
tightened to guarantee real wall clearance, not just closing the LAND
gap):
1. **`geofence_hard_limit_m`** (new required param) — an absolute x/y
   clamp on `_geofence_bounds()`, independent of mission geometry.
   `3.75` in sim (derived from `vio_test.sdf`'s actual ±4.75m fence,
   guaranteeing >=1m clearance); `10.0` UNTESTED placeholder in hw
   (no fixed real test-area geometry exists yet to derive a real number
   from — flagged loudly in-file, must be retuned before free flight).
2. **`_check_geofence(during_land=True)`**, now also called from
   `FlightState.LAND`. Deliberately a DIFFERENT response than the normal
   path: `_land()` is a no-op there (already the active mode), so this
   is a genuine last resort — force-disarm immediately
   (`_disarm()`, reused). At 2m mission altitude, judged an uncontrolled
   drop safer than continuing toward/through a wall. Ends the flight
   (`FlightState.DONE`), no recovery attempt — human inspection expected
   after this fires.

**Caught and fixed a real bug in this fix while building it, before ever
testing**: the two response paths originally shared ONE latch flag
(`_geofence_breached`) — meaning a mid-cruise breach (normal path,
transitioning to LAND) would silently BLOCK the new during-land
emergency check from ever firing if `AUTO_LAND` then ALSO diverged
further during the resulting descent — exactly the compounding failure
this exists to catch. Split into two independent latches
(`_geofence_breached` / `_geofence_land_emergency`) before testing.

**Verified live, not just code-reviewed**:
- Normal regression (plain GPS `square`, no interference): clean before
  AND after every change in this fix — the new hard limit doesn't
  false-trigger at this mission's normal geometry (±3.5 bbox+margin vs
  3.75 hard limit).
- The actual mechanism: two flawed attempts to force a LAND-phase-only
  breach via Gazebo's `set_pose` teleport service both landed too late
  (real GPS landings complete in ~7s; by the time a docker-exec-dispatched
  teleport arrived, the vehicle had often already disarmed) — a real
  testing-methodology lesson, not a code bug, documented so a future
  session doesn't repeat it.
- The test that DID work, and is arguably the more important one anyway:
  teleported the vehicle out of bounds mid-WAYPOINTS. Normal path fired
  (`GEOFENCE BREACH`, transitioned to LAND) — and on the VERY NEXT
  control tick (100ms later), the independent during-land check ALSO
  fired (`GEOFENCE BREACH DURING LANDING`), force-disarmed with the
  correct reason string logged, and the process exited cleanly. This is
  exactly the compounding scenario the two-latch fix targets, confirmed
  working end to end, not just in isolation.

Full write-up: README §12 issue 33 (safety fix) and issues 29-32 context
for the still-open accuracy question.

**Deferred at the time — BUILT THE SAME SESSION, see part 18 below**: the
user also asked about a broader in-flight vision-HEALTH watchdog.

## Phase 3, part 18 (2026-07-13, later same day) — estimate-health watchdog built, triggered by a REAL live crash (not hypothetical)

User pasted a terminal traceback from their 3rd square-mission test run:
`openvins_odometry_bridge` crashed outright —
`RuntimeError: Unable to convert call argument to Python object`, raised
INSIDE rclpy's own `executors.py::_take_subscription` (`sub.handle.
take_message(...)`), i.e. BEFORE the node's own `_on_odometry` callback
is ever reached. Checked `docker stats`/`uptime` at the time: host load
average 15.2, `rviz2` alone at 63% CPU (GPU rendering), `px4-sim` at
215%, plus an unrelated `robot_ws_dev` container also running — severe
resource contention. This matches a known category of rclpy/CycloneDDS
fragility under CPU starvation (corrupted/dropped packet reassembly at
the DDS layer), not a logic bug in the bridge's own Python code. The
mission still landed only mildly drifted that time — pure luck, nothing
was watching for this class of failure at all.

Given the user just hit exactly the scenario the previously-deferred
"vision health watchdog" was meant for, asked (via AskUserQuestion) and
confirmed: build it now rather than defer again.

**Two fixes, both real, both needed together**:

1. **Bridge resilience** (`openvins_odometry_bridge.py`): `rclpy.spin(node)`
   -> a manual `while rclpy.ok(): try: rclpy.spin_once(...); except
   RuntimeError: log and continue` loop. Plain `spin()` lets ANY uncaught
   exception kill the whole process; this survives a single transient DDS
   hiccup. Explicitly documented as NOT a root-cause fix (host contention
   is) and as only protecting THIS node — doesn't help if the same
   condition kills `camera_imu_bridge` or `run_subscribe_msckf` instead.

2. **Estimate-health watchdog** (`offboard_control_node.py`, new
   `_check_estimate_health` + `estimate_invalid_abort_dwell_s` param) —
   the general fix. Deliberately reads PX4's OWN
   `VehicleLocalPosition.xy_valid`/`v_xy_valid` rather than anything
   vision-specific (no new subscription needed — `VehicleLocalPosition`
   was already fully subscribed) — keeps `offboard_control_node.py`
   unaware of whether GPS or vision is active, preserving the
   `common_perception` package's own stated invariant
   ("common_control/common_missions never know which source is active").
   This catches the crashed-bridge scenario AND any other cause of a
   genuinely untrustworthy estimate, source-agnostic. Same
   TAKEOFF/WAYPOINTS/HOVER/LAND coverage, same during-land
   force-disarm-as-last-resort shape, and the SAME independent two-latch
   pattern (`_estimate_breach` / `_estimate_land_emergency`) as part 17's
   geofence fix, for the identical compounding-breach reason. One
   difference: a short dwell (debounces a single-tick `xy_valid` flicker,
   unlike the geofence's deliberately-immediate check).

**Real bug found and fixed while wiring the params, before testing
anything**: `hw_params.yaml`'s `square_mission` section was missing
`geofence_hard_limit_m` entirely — part 17 only added it to the hover
profile's section by mistake. Would have failed loudly at startup (no
silent param defaults anywhere in this project), not silently
misbehaved, but still a real oversight. Fixed alongside adding the new
`estimate_invalid_abort_dwell_s` to both hw sections.

**Verified live, twice, deliberately different scenarios**:
- Killed GPS fusion via MAVLink (`EKF2_GPS_CTRL=0`, a throwaway
  `kill_gps.py` script using the same PARAM_SET pattern as
  `set_localization_source.py`) mid-`square`-mission, vision not
  running either (true "GPS is not there" — the user's other direct
  question). Dead-reckoning drift reached the geofence bound BEFORE the
  estimate-health dwell elapsed — geofence caught it first, both systems
  worked correctly together, force-disarmed during the resulting
  AUTO_LAND.
- To isolate the NEW check specifically (prove it fires independent of
  the geofence, not just riding along): killed GPS mid-`hover` instead
  (vehicle not commanded to move -> slower, smaller drift) with a
  test-only extended hover duration (a temp params file,
  `hover_seconds: 40`, launched directly via `common_missions
  autonomy.launch.py` bypassing `sim_bringup` to inject it) so the
  watchdog had time to react before the mission would otherwise land
  normally. `xy_valid`/`v_xy_valid` went false ~6.7s after the kill
  (PX4's own 5s NOAID_TOUT + this fix's 1.5s dwell, numbers line up),
  zero geofence involvement, clean abort-to-LAND, and the independent
  during-land check fired on the very next control tick and
  force-disarmed. Confirms the new mechanism works on its own, not just
  riding on the geofence's coattails.

Full write-up: README §12 issue 34.

## Phase 3, part 19 (2026-07-14) — user-reported "stuck waiting for arm/offboard forever" root-caused to a forgotten `PX4_GZ_WORLD=vio_test`, NOT a code bug; fail-loud guard added so it can't silently recur

Resumed session found an uncommitted, untested diff already sitting in
`set_localization_source.py` from the tail end of 2026-07-13 (raising
vision-mode `EKF2_NOAID_TOUT` to PX4's max 10s — see that file's own
docstring) — confirmed it applies cleanly and is picked up live
(`EKF2_NOAID_TOUT = 10000000 (confirmed)` in every repro this session),
but it turned out to be unrelated to the bug the user actually hit.

User pasted a log: `square_mission` repeating `Arm command sent` /
`Waiting for offboard+armed (nav_state=14, arming_state=1)` with no
apparent end. Live repro (`make sim` + `make mission MISSION=square
LOCALIZATION=vision VIO_BACKEND=openvins`) reproduced a problem, but not
literally that exact symptom the first time — instead PX4 armed almost
instantly, climbed, then issue 34's estimate-health watchdog aborted
to LAND ~4s later (`POSITION ESTIMATE INVALID`). Root-caused via a
targeted experiment: launched `common_perception vio.launch.py` in
isolation (no mission, vehicle stationary) and watched `run_subscribe_
msckf`'s own log for a full 20s+ — zero output, not even initialization
progress. `ros2 topic hz /camera/camera/imu` and `/camera/camera/color/
image_raw` both timed out with literally zero messages. Checked Gazebo's
own topic tree (`gz topic -l` inside `px4-sim`) and found the mismatch
immediately: TWO parallel model namespaces exist,
`/world/empty/model/x500_depth_0/...` and `/world/vio_test/model/
x500_depth_0/...` — `ros_gz_bridge`'s config
(`config/ros_gz_bridge.yaml`) is hardcoded to the `vio_test` names, but
`make sim` had been started with no `PX4_GZ_WORLD` override, defaulting
to `empty` (documented default, correct for GPS-mode). Confirmed with a
direct `gz topic echo` on both: `vio_test`'s IMU topic had real live
data, `empty`'s (well, the *absence* of a `vio_test` publisher) had
none. This is `README §4.3/§8`'s already-documented "`VIO_BACKEND=
openvins` needs `PX4_GZ_WORLD=vio_test`" requirement — not a new
discovery that it's needed, but a NEW discovery of exactly how silently
it fails when forgotten, and that the failure has (at least) two
different-looking symptoms depending on arm-time timing luck, not two
different bugs.

**Decisively confirmed via a controlled A/B in the same sim session**,
changing only `PX4_GZ_WORLD`: `vio_test` → mission armed, climbed, flew
all 5 waypoints, landed, issue 21's post-landing low-throttle fallback
disarm handled the still-open post-landing-drift gap exactly as
designed. `empty` → reproduced the failure (this time as the quick-abort
variant, not the forever-stuck variant — same root cause, different
timing-dependent presentation, confirmed by re-reading both mission
logs side by side).

**Fix**: rather than just telling the user to remember the flag (already
documented, already forgotten once), added `camera_data_check` as a
fourth concurrent action in `vio.launch.py` — `timeout 8 ros2 topic
echo /camera/camera/imu --once`, and on failure logs one unmissable
`[camera_data_check] ERROR` line naming the cause and the exact fix
command. Doesn't gate or delay the other three actions (matches the
file's existing "no strict sequencing needed" design). Verified live:
re-ran the exact `empty`-world repro after adding the guard — the new
error line appeared within 8s, pointing straight at `PX4_GZ_WORLD=
vio_test make sim`.

**Noticed but NOT investigated further, does not block flights**:
`run_subscribe_msckf` segfaults (exit -11) on every run this session,
but only during the launch's own SIGINT/shutdown sequence, after the
mission already landed or force-disarmed. Never observed affecting an
in-progress flight. Flagged in README issue 35 so a future session
doesn't mistake it for a new in-flight regression.

Full write-up: README §12 issue 35.

## Phase 3, part 20 (2026-07-14, later same day) — user flagged the `vio_test` hardcoding itself as bad practice; fixed properly, and verification caught a real blind spot in part 19's own guard

User read the part 19 diff and pushed back on the actual root object,
not just the symptom: `ros_gz_bridge.yaml`'s Gazebo-side topic paths
were still a literal `/world/vio_test/model/x500_depth_0/...` string —
issue 35's fix made a MISMATCHED world fail loudly, but didn't stop the
mismatch from being possible by construction. Asked for a runtime
variable sourced from the same command-line world selection, or a
better practice than hardcoding — explicit invitation to redesign, not
just patch.

**Design chosen**: turned `ros_gz_bridge.yaml` into a template
(`__WORLD__`/`__MODEL__` tokens). `vio.launch.py` gained `world`/`model`
launch arguments; `world` defaults to reading the `PX4_GZ_WORLD`
environment variable via `launch.substitutions.EnvironmentVariable` —
deliberately the SAME variable already used on the `make sim`/`make
sim-gui` command line to pick the actually-running world, not a second
independently-settable value, specifically to prevent the two from ever
disagreeing again. Required one more fix to actually reach the ROS
graph: `docker-compose.yml`'s `ros2-autonomy` service never had
`PX4_GZ_WORLD` in its `environment:` block at all (only `px4-sim` did) —
added it, confirmed live (`docker exec ros2-autonomy bash -c 'echo
$PX4_GZ_WORLD'` before touching anything else). New
`_bridge_config_with_world` function (an `OpaqueFunction`, since the
resolved string value is needed before `parameter_bridge` starts, not
just as a launch-graph substitution) reads the template, does plain
`str.replace()` on the two tokens, writes the result to a `/tmp` file,
and points `parameter_bridge`'s `config_file` at that — `config_file` is
loaded as static YAML with zero substitution support of its own, so
this has to happen in Python before the node ever launches. Considered
and rejected switching to `ros_gz_bridge`'s CLI-argument bridging
syntax instead of YAML (would avoid the generate-a-tempfile step) —
rejected specifically because its exact syntax varies across
`ros_gz_bridge` versions and this project pins versions carefully
elsewhere; regenerating the already-verified YAML mechanism from a
template kept the change to "where the config comes from," not "how
bridging works," which is the lower-risk option for a flight-critical
sensor bridge.

**A real gap in part 19's OWN guard was found DURING verification, not
assumed** — exactly the kind of thing this project's "always verify
live" rule exists to catch. Reran the identical empty-world repro that
originally proved the guard worked, and it now passed cleanly instead
of firing — investigated rather than shrugged off, and the reason
turned out to be structural, not a fluke: once `world` always matches
the real running world, `/camera/camera/imu` (the topic part 19's guard
watched) gets real data on ANY world including `empty`, because an IMU
is a physics sensor with no dependency on ground texture. The guard's
input-presence check was only ever a proxy for "will OpenVINS
initialize," and fixing the topic-mismatch bug silently invalidated
that proxy — the guard would have gone quiet on exactly the *other*
already-documented VIO requirement (visual texture, `ros_gz_bridge.yaml`
had noted this since Milestone B: `empty`'s flat ground gives zero
trackable corners) without anyone noticing until a future real flight
hung again with no explanation, right back to square one. Fixed by
renaming to `vio_output_check` and repointing it at OpenVINS's own
output topic (`/ov_msckf/odomimu`) instead of its input — checks the
thing that actually matters (did a valid estimate ever come out)
regardless of which upstream cause is at fault, and covers both known
causes (wrong world, or a world with data but insufficient texture)
with one mechanism. Timeout raised 8s → 15s to tolerate a legitimately
slower-but-fine static init now that "no data at all" (fast, obvious
failure) and "data present but never initializes" (needs real init time
before you can call it failed) are both in scope for the same check.

**Verified live, three separate runs, same session, isolating each
change**: (1) `vio_test` with the new template rendering — full mission
success, bridge's own startup log confirmed the CORRECTLY rendered
`/world/vio_test/model/x500_depth_0/...` paths (not the template
tokens). (2) `empty` with the OLD `/camera/camera/imu`-based check
still in place — check passed cleanly (confirming the blind-spot theory
directly, not just in theory) while the mission still failed identically
to part 19 (`POSITION ESTIMATE INVALID`, force-disarm) — this run is
what surfaced the gap. (3) `empty` again after switching to the
`/ov_msckf/odomimu`-based check — fires correctly within 15s, message
names both possible causes. Re-ran `vio_test` a fourth time after the
check change specifically to confirm no false-positive on the good path
(OpenVINS's real init time is well under 15s here, as expected).

Full write-up: README §12 issue 36.

## Phase 3, part 21 (2026-07-14, later same day) — the shutdown SIGSEGV finally analyzed properly, not just shrugged off; root cause is upstream rclcpp, confirmed benign

User pasted a fresh `make flight-test LOCALIZATION=vision
VIO_BACKEND=openvins` log — the mission itself landed and disarmed
cleanly, but asked for an actual analysis of the `run_subscribe_msckf`
SIGSEGV that's been showing up (and getting deferred) since part 19.

**First attempt: tried to catch it live under gdb, learned something
useful about this project's own tooling in the process, not just the
crash.** Installed gdb into `ros2-autonomy` (not present by default),
attached to a live `run_subscribe_msckf` PID mid-hover, set `handle
SIGINT/SIGTERM nostop noprint pass` so the process's own signal
handling would proceed normally, and waited for the mission to finish
so gdb could catch the crash with a real backtrace. **The attach
silently failed** — `gdb -p <pid>` from a `docker exec` shell cannot
ptrace a process it didn't spawn under Docker's default seccomp/
capability profile (needs `CAP_SYS_PTRACE`, not granted here) — and
critically, this failure was NOT visible until minutes later, because I
was polling a log file for mission completion rather than gdb's own
output. Real lesson, not just a wasted attempt: while waiting, host
load climbed to **18** (my own `apt-get`/`gdb`/repeated `docker exec`
overhead stacking on the user's already-running `rviz2`+`rqt-viewer`
GUI session) and the mission got stuck in the exact "waiting for
offboard+armed" loop from part 19 — but this time confirmed as a REAL
resource-contention artifact (matching issue 34's documented category
exactly), not a regression: `PX4_GZ_WORLD` was correctly `vio_test`,
`arming_check_error_flags` decoded (via `health_report` +
`common_with_enums.json`'s bit-to-name mapping, README issue 4's own
debugging tip) to `local_position_estimate` + `global_position_estimate`
+ `system` (PX4's own CPU/RAM health flag) all failing simultaneously —
i.e., PX4 itself was reporting the host as overloaded. Killed the stuck
run and my own gdb/apt overhead; load dropped from 18 to ~4 within
seconds, confirming the diagnosis. **Lesson for future sessions: don't
stack heavy debugging tooling (gdb, apt-get, repeated docker exec) on
top of a host already running the GUI stack** — it can manufacture a
symptom that looks exactly like a real regression.

**Second attempt, much cheaper and actually worked: read the source
instead of fighting the debugger.** OpenVINS is built from source in
this image (`docker/Dockerfile.ros2_autonomy`'s `colcon build
--packages-select ov_core ov_init ov_msckf ov_eval`), so the vendored
upstream source is sitting right there:
`/opt/openvins_ws/src/open_vins/ov_msckf/src/run_subscribe_msckf.cpp`.
Its `main()`: `rclcpp::executors::MultiThreadedExecutor executor;
executor.spin();` (blocks until SIGINT), immediately followed by
`viz->visualize_final()` (reads shared `VioManager`/`ROS2Visualizer`
state — camera intrinsics, IMU-camera timeoffset, extrinsics) and
`rclcpp::shutdown()`. `MultiThreadedExecutor::spin()` returning on the
calling thread doesn't guarantee every worker thread it spawned has
actually finished touching node state — a known, still-open category of
`rclcpp` SIGINT/executor-thread races (a quick web search turned up
several open upstream `ros2/rclcpp`/`ros2/launch` issues describing the
same shape) — so a callback still in-flight on another executor thread
can race `visualize_final()`'s read of the same state on the main
thread. Confirmed this ISN'T just a plausible theory: the two lines that
appear right before every single crash in every log this project has
ever captured —
```
camera-imu timeoffset = 0.01667
cam0 intrinsics = 465.922,466.035,320.933,179.928 | 0.001,0.000,0.004,0.002
```
are `visualize_final()`'s own `PRINT_INFO` calls, verified
character-for-character against the source. The crash happens inside or
immediately after this exact function — exactly where the race theory
says it should.

**Decision: documented, not patched.** This project doesn't fork/patch
upstream repos ([[drone-project-workflow-prefs]]), and the actual
fragility lives inside `rclcpp`'s own executor/signal-handling
internals — not something a few lines of local patching to vendored
OpenVINS source would reliably fix, for a crash that has never once
appeared before `Landed and disarmed — mission complete` across 7+
reproductions this session (including the user's own). Full write-up
in README issue 37.

Full write-up: README §12 issue 37.

## Phase 3, part 22 (2026-07-14, later same day) — README split into a modular doc set; no content deleted, only relocated

User feedback: the single-file README had grown to 1831 lines, over half of
it (§12 Known Issues, ~885 lines) — too much for a first-time reader to get
through to find the Quick Start. Asked for a "professional, modular" README:
essentials on the main page, everything else linked out from `resource/`.

Restructured into 5 files, all content preserved (verified — see below):
- `README.md` (1831 → 264 lines): tagline, Objective, Architecture, a NEW
  condensed Quick Start (clone → build → sim → build-ws → flight-test, one
  block), a NEW Documentation index table, Repository Layout, No-Fork
  policy, Roadmap.
- `resource/setup-guide.md`: old §3 Prerequisites + §4.1-4.4 (clone, build
  images, start sim, build workspace).
- `resource/mission-testing.md`: old §4.5-4.9 (wait for PX4, hover test, fly
  a mission, inspect the system, stop the stack).
- `resource/reference.md`: old §5-§10 (commands, topics, params,
  localization source, launch files, env vars) — merged into one file
  rather than five tiny ones, each subsection still separately anchored.
- `resource/known-issues.md`: old §12 verbatim (all 37 issues), plus a new
  index/TOC linking to each by number.

**Mechanically extracted (line-range `sed`), not retyped**, specifically to
avoid transcription risk across ~1800 lines — verified by diffing a
normalized (markdown-link-and-anchor-stripped) version of each new file
against the original section it came from; every diff found was manually
inspected and confirmed to be either a stable anchor tag insertion or a
deliberate link-text improvement (e.g. "§12 issue 4" → "issue 4" once the
link target already encodes "12"), never a content change.

**Cross-references were the hard part, not the extraction.** ~82 "§N"
references are scattered through the original document (e.g. "§12 issue
33", "§4.3, §6", "§12, issues 17-25") pointing at sections that no longer
exist as numbered sections once split across files. Wrote a small Python
regex-based fixer (`fix_refs.py`, scratch-only, not committed) rather than
hand-editing ~82 spots — assigns every old section number a
(target-file, anchor) pair and rewrites in place. **Caught and fixed a real
bug in the fixer itself before trusting its output**: an early regex
version had an unused second capture group meant for patterns like
"§4.3, §6" — it matched both references but the replacement logic only
used the first, silently DELETING the second one and everything between
them. Caught via the same normalized-diff verification pass (not by
assuming the regex was correct), fixed by simplifying to one match per
`§N` occurrence, re-ran, re-verified. Also wrote and ran a link/anchor
validator (walks every markdown link across all 5 files, confirms the
target file exists and the anchor is actually defined there) — caught two
more real mistakes: a stale anchor (`#11-no-upstream...` after the target
heading was renamed without its old "11." prefix) and one bare
same-page `#issue-17` link sitting in `reference.md`, where `#issue-17`
doesn't exist (it needed `known-issues.md#issue-17`).

**Verified, not assumed, that nothing was lost**: word count went UP
(15607 → 16880 across the new 5-file set) since new navigation content was
added on top of full preservation; a 40-sentence random sample of the
original, normalized, was checked for verbatim presence somewhere in the
new set — all 4 "misses" were manually confirmed as intentional rewording
in the hand-composed new sections (Quick Start, Documentation index,
Architecture's link text), not lost content.

Not pushed yet — commit only, per standing workflow preference
([[drone-project-workflow-prefs]]); user will push.

## Phase 4, part 1 (2026-07-14, later same day) — user shared real airframe details (custom quad, Pixhawk 6C, Orange Pi 5 Plus/Ubuntu 22, D435i, external GPS) and asked for a step-by-step bring-up guide; built the missing infra so the guide would actually be followable, not just aspirational

User's ask: analyze the repo's real-hardware readiness, advise on camera
calibration, and produce a GPS-phase/VIO-phase step-by-step guide under
`resource/`, linked from the README.

**Investigation first**: `hw_bringup` (launch file + `hw_params.yaml`)
already existed as a well-documented stub, but `docker-compose.yml` had
NO `hw` profile, no `Dockerfile.hw_*` existed at all, and `Makefile` had
zero hw targets — `IMPLEMENTATION_PLAN.md`'s own Phase 4 checklist
confirmed all of this was still unchecked. A guide referencing
`make build-hw` or similar would have been fiction. Also found:
`hw.launch.py`'s VIO slot was a literal commented-out placeholder (never
wired in), and `set_localization_source.py`'s vision-mode lever arm
(`VISION_LEVER_ARM_FRD`) was hardcoded to the SIM d435i model's
co-located (near-zero) offset with no override — wrong by construction
for a different physical airframe, and would have silently applied the
sim's geometry to real hardware if not caught.

**Decision: build the infrastructure, not just document around the
gaps.** Justified by existing project precedent — `hw_bringup` itself was
already an accepted pattern of "ship well-reasoned but untested hardware
scaffolding, clearly marked, to be validated when hardware arrives"; this
extends that same pattern rather than inventing a new one. Built:
- `docker/Dockerfile.hw_autonomy` — ARM64, mirrors
  `Dockerfile.ros2_autonomy` (same DDS agent/px4_msgs/px4_ros_com/
  OpenVINS) minus the Gazebo-only `ros_gz_bridge` layer, plus
  `librealsense2` built from source with `-DFORCE_RSUSB_BACKEND=ON`
  (Intel's apt repo has no ARM64 packages; RSUSB avoids needing a
  patched host kernel UVC driver, consistent with this project's own
  no-host-install policy) and `realsense-ros` built against it. ONE
  combined image, not the two-container split
  (`Dockerfile.hw_sensors` + a separate DDS-agent image) the original
  `IMPLEMENTATION_PLAN.md` checklist had sketched — no benefit splitting
  them on a single companion computer, real cost in cross-container DDS
  discovery complexity for no reason. Deviation documented in the
  Dockerfile's own header, not silently substituted.
- `docker-compose.yml`'s new `hw` profile (`hw-autonomy` service) —
  `privileged: true` + `/dev/bus/usb` mount (RSUSB needs to enumerate
  USB topology itself, not a fixed pre-guessed node) + the Pixhawk
  serial device. Separate `hw_colcon_build_cache` volume (ARM64 build
  output isn't interchangeable with sim's).
- `Makefile` hw targets mirroring the sim ones exactly
  (`build-hw`/`build-ws-hw`/`hw-flight-test`/`hw-mission`/`shell-hw`/
  `stop-hw`), plus new `MAVLINK_URL`/`EV_POS_X/Y/Z` override variables
  (see below) — all with loud "UNTESTED, see the guide first" comments,
  not silently presented as equivalent-to-sim.
- `hw.launch.py`: replaced the placeholder comment with an actual
  conditional include of a new `common_perception/launch/hw_vio.launch.py`
  when `localization_source:=vision` — realsense-ros + OpenVINS +
  `openvins_odometry_bridge` (that last node reused completely unmodified;
  confirmed by reading it that it has zero sim-specific logic) + a
  `vio_output_check` guard reusing the exact same fail-loud design as
  sim's own (issue 35/36) but watching `/ov_msckf/odomimu` from the
  start this time, not repeating the input-vs-output mistake that guard's
  own history already went through once.
- Three new OpenVINS calibration files
  (`estimator_config_hw.yaml`/`kalibr_imucam_chain_hw.yaml`/
  `kalibr_imu_chain_hw.yaml`) — explicit templates with `FILL IN` markers,
  not fabricated-plausible-looking numbers. One exception: IMU noise
  density values ARE filled in, but sourced from a cited published paper
  (Sier et al., arxiv 2504.14376) on the D435i's BMI055, found via a live
  web search specifically because seeding an unverified specific decimal
  into a safety-relevant config uncited felt like the wrong call — cited
  in the file itself so the user can verify the source, not just trust
  the number. `calib_cam_extrinsics: true` (NOT sim's frozen `false`) —
  sim could only freeze its value after independently re-deriving AND
  flight-confirming it (see part with issues 22/23); a raw Kalibr output
  has no such cross-check yet, so freezing it here would risk repeating
  the exact bug that froze-too-early already caused once in sim.
- `set_localization_source.py`: added `--ev-pos-x/y/z` CLI overrides
  (optional, default `None` → sim behavior unchanged), used by
  `hw.launch.py`'s new `ev_pos_x/y/z` launch arguments (deliberately
  `0.0` sentinel defaults, not the sim value — an unmeasured real mount
  should fail obviously wrong, not silently inherit a different
  airframe's geometry).

**Wrote the two guides** (`resource/hardware-bringup-gps.md`,
`resource/hardware-bringup-vio.md`) as the actual step-by-step deliverable
— GPS phase deliberately isolates `hw_bringup`'s own unverified risk from
VIO's separately-unverified risk (mirrors this project's own standing
principle of isolating unknowns one at a time, e.g. Milestone A's
loopback stand-in proving the switch mechanism before real VIO). VIO
guide includes a real Kalibr camera-IMU calibration procedure (sim never
needed this — its extrinsics were analytically derivable from known SDF
geometry, which doesn't exist for a hand-built real mount) and explicit
warnings against repeating the tilted-camera-rig's still-unresolved 84m
divergence (`dev/camera-tilt` branch, referenced but never merged).

**Verified what could be verified without real hardware**: all three new
YAML files parse; `docker compose config` validates the new `hw` profile;
all touched/new Python and launch files pass `ast.parse`; a link/anchor
validator (same one built for the README split) confirms every link
across all 7 doc files (README + 6 resource docs) resolves. What could
NOT be verified: anything requiring actual hardware — no ARM64 build, no
real USB/serial enumeration, no real flight. This is explicitly flagged
throughout both guides and in every new file's own header, not glossed
over.

Not committed yet as of this entry — pending user review.

## Explicitly not done yet (don't assume otherwise)
- **Part 14 above (2026-07-10): the tilted rig is NOT flight-ready — one
  real flight, one 84m divergence crash (open). Do NOT hand the user the
  5-run accuracy procedure as if it were ready. Also: the in-flight vision
  watchdog (abort-to-LAND if vision stops/never arrives post-arming) is
  designed but NOT built — user explicitly deferred it until after
  accuracy testing.**
- ~~Part 13's "test on an idle host" mitigation~~ — SUPERSEDED by part
  14's /dev/dri fix (the host was never the bottleneck; the missing GPU
  mapping was).
- **Real OpenVINS VIO exists, genuinely fuses into EKF2 GPS-denied, and TWO
  confirmed root-cause extrinsics bugs are now fixed** — see "Phase 3
  Milestone B result" parts 2 and 3 above. A `square` mission now flies all
  4 waypoints accurately and survives the fast `AUTO_LAND` descent that used
  to diverge. **But a THIRD, currently OPEN issue blocks unattended use**:
  post-landing, PX4's position/velocity estimate keeps drifting (physical
  vehicle is fine — confirmed via Gazebo ground truth — only the estimate
  runs away), so `landed` never goes true and auto-disarm never fires.
  **Every `openvins` flight currently needs a manual
  `px4-commander disarm -f` after landing** — do not run
  `make mission ... VIO_BACKEND=openvins` unattended until this is fixed
  (leading theory: `zupt_only_at_beginning: true` leaves nothing correcting
  drift once stationary post-landing — see part 3 for detail). Milestone
  A's `loopback_odometry_bridge` (zero-drift stand-in) remains the only
  vision option that auto-disarms reliably today.
- `ros_gz_bridge` for the simulated camera/IMU IS configured and verified
  live (`config/ros_gz_bridge.yaml`, confirmed against a running
  `PX4_GZ_WORLD=vio_test` SITL via `gz topic -l`) — now bridging our own
  `d435i` model's sensors (color/depth/imu), not the stock `OakD-Lite`'s.
- No Nav2/SLAM (Phase 5).
- No real hardware bring-up has been RUN — as of Phase 4 part 1
  (2026-07-14), `hw_bringup`/`hw-autonomy`/`hw_vio.launch.py` all exist
  and are internally consistent (Python/launch files parse, YAML parses,
  `docker compose config` validates, link validator confirms the two new
  guides), but NONE of it has executed on ARM64 hardware or against a
  real Pixhawk — everything is "written and reviewed," not "verified."
  See resource/hardware-bringup-gps.md / hardware-bringup-vio.md for the
  actual step-by-step path and resource/known-issues.md for where to log
  what breaks first. `mavlink_url`'s default is still SITL-shaped by
  design (a required-looking placeholder, not a real value) — the
  Makefile's `MAVLINK_URL` variable and the GPS guide's §3 cover setting
  it correctly per-deployment.
- No YAML params file for `hw_bringup`'s values has been tuned against real
  flight — `hw_params.yaml` is untested guesses (smaller/conservative), not
  validated numbers.
- `common_control`/`common_missions` are confirmed genuinely unmodified by
  all of Phase 3 Milestone A — verified by reading the actual diff, not just
  asserted.
