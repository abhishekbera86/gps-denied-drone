# Known Issues & Fixes

> Part of the [GPS-Denied Autonomous Drone Stack](../README.md) documentation set.

Every real bug, gotcha, and dead end hit building and flying this stack, in the
order encountered — documented so a future rebuild-from-scratch (or a fresh
pair of eyes) doesn't waste time rediscovering them. Almost all of these are
already fixed in this repo; the few that aren't (e.g. issue 37) were
root-caused and confirmed benign rather than patched, and say so explicitly.

## Index

1. [`DONT_RUN=1` does not stop the new Gazebo target from launching.](#issue-1)
2. [`ros:humble-ros-base` doesn't ship `rmw_cyclonedds_cpp`.](#issue-2)
3. [Don't use `px4io/px4-sitl-gazebo` or similar prebuilt PX4+Gazebo images without checking their pi...](#issue-3)
4. [PX4 refuses to arm the `x500_depth` SITL model out of the box.](#issue-4)
5. [Restarting only the px4-sim container wedges the DDS bridge.](#issue-5)
6. [PX4 v1.17 publishes versioned topic names.](#issue-6)
7. [The offboard signal = heartbeat + setpoint stream, and commands need retries.](#issue-7)
8. [Arming is rejected for the first ~30–60 s after `make sim` — this is normal.](#issue-8)
9. [On a weak/older CPU the simulation runs slower than wall-clock.](#issue-9)
10. [If the first `make build` dies mid-way (network hiccup), just rerun it.](#issue-10)
11. [Both containers use host networking — check for conflicts on shared machines.](#issue-11)
12. [Never put an inline `# comment` after a value in `.env`.](#issue-12)
13. [`HEADLESS=0` does NOT enable the Gazebo GUI — it must be unset.](#issue-13)
14. [PX4's uXRCE-DDS bridge cannot set PX4 parameters at all.](#issue-14)
15. [Humble's default `ros-humble-ros-gz-bridge` targets the wrong Gazebo version.](#issue-15)
16. [A `RUN ... && rm -rf /var/lib/apt/lists/*` layer wipes the apt cache for every subsequent `RUN` l...](#issue-16)
17. [Real VIO's cruise speed matters more than it looks — too slow starves scale/bias observability.](#issue-17)
18. [`EKF2_EV_POS_X/Y/Z` (the vision sensor's lever arm) was never configured — a genuine, previously-...](#issue-18)
19. [Setting `PX4_GZ_WORLD` in `.env` silently defeats the `PX4_GZ_WORLD=vio_test make sim-gui` comman...](#issue-19)
20. [The drone can be missing from the Gazebo GUI's 3D view/entity tree even though it's simulating co...](#issue-20)
21. [Post-landing, PX4's own position/velocity ESTIMATE can keep drifting even though the vehicle is p...](#issue-21)
22. [Wrong VIO camera-IMU extrinsic rotation caused a real in-flight divergence — twice, for two diffe...](#issue-22)
23. [Monocular VIO lost all trackable features near the ground — `vio_test.sdf`'s ground plane was fla...](#issue-23)
24. [A `square` mission flew out of the fenced/textured area at speed and hit the ground — a real cras...](#issue-24)
25. [While reproducing issue 24, found a second, worse bug: a flight/mission node's OS process can out...](#issue-25)
26. [Manually launching an image viewer (`rqt_image_view`) per mission is a sequencing race, not just ...](#issue-26)
27. [`px4_msgs`' generated Python fields (e.g. `VehicleOdometry.position`, `.q`) are numpy `float32` a...](#issue-27)
28. [`VIO_BACKEND=openvins` flights were genuinely inconsistent flight to flight — accurate on one `sq...](#issue-28)
29. [Arming-reliability gap found verifying the 3 fixes above, then fully ROOT-CAUSED the same day — a...](#issue-29)
31. [`/dev/dri` (GPU rendering) backported to the headless base `docker-compose.yml` (2026-07-13) — tu...](#issue-31)
32. [`/drone/path` (RViz) reset on arm (2026-07-13)](#issue-32)
33. [`FlightState.LAND` had NO geofence check at all — confirmed as the direct cause of a real vehicle...](#issue-33)
34. [In-flight estimate-health watchdog — built the same day issue 33 was fixed, after a concrete, liv...](#issue-34)
35. [`VIO_BACKEND=openvins` silently starved of all camera/IMU data when `PX4_GZ_WORLD=vio_test` is fo...](#issue-35)
36. [Follow-up to issue 35: `vio_test` was still hardcoded into `ros_gz_bridge.yaml`'s topic paths — f...](#issue-36)
37. [`run_subscribe_msckf` (OpenVINS's own process) segfaults on every shutdown — root-caused to upstr...](#issue-37)

---

Documenting these so a future rebuild-from-scratch doesn't waste time
rediscovering them — all are already fixed in this repo.

<a id="issue-1"></a>

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

<a id="issue-2"></a>

**2. `ros:humble-ros-base` doesn't ship `rmw_cyclonedds_cpp`.**
Setting `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` without installing the
matching package fails with `librmw_cyclonedds_cpp.so: cannot open shared
object file`. Fixed in `docker/Dockerfile.ros2_autonomy` by explicitly
installing `ros-humble-rmw-cyclonedds-cpp` (appended as its own `RUN` layer
*after* the expensive `colcon build` of `px4_msgs`/`px4_ros_com`, so editing
it doesn't invalidate that ~10-minute build cache).

<a id="issue-3"></a>

**3. Don't use `px4io/px4-sitl-gazebo` or similar prebuilt PX4+Gazebo images
without checking their pinned PX4 version first.** One such image already
cached on this project's dev machine turned out to ship PX4 v1.18.0-alpha1
(an unreleased alpha) with no path back to a stable version — its apt repo
config had already been stripped from the image, leaving only the alpha
`.deb` installed. Always check `apt-cache policy px4-gazebo` (or equivalent)
before trusting a prebuilt image's version claim.

<a id="issue-4"></a>

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

<a id="issue-5"></a>

**5. Restarting only the px4-sim container wedges the DDS bridge.** If
px4-sim is recreated while ros2-autonomy (and its Micro-XRCE-DDS-Agent) keeps
running, the session "re-establishes" in the agent log and datawriters get
recreated — but no data flows, and `px4-uxrce_dds_client status` inside PX4
reports "Running, disconnected" with timesync never converging. Restarting
the agent process alone did not recover it either. **Workaround: always
restart the full stack together** (`make stop && make sim`).

<a id="issue-6"></a>

**6. PX4 v1.17 publishes versioned topic names.** Messages that carry a
`MESSAGE_VERSION` field (e.g. `VehicleStatus`, `VehicleLocalPosition`) appear
on the wire as `/fmu/out/vehicle_status_v1`, `/fmu/out/vehicle_local_position_v1`
— not the unversioned names used in most PX4 examples and docs. Subscribing
to the unversioned name silently receives nothing. `common_control` already
uses the `_v1` names.

<a id="issue-7"></a>

**7. The offboard signal = heartbeat + setpoint stream, and commands need
retries.** PX4 only treats the offboard link as "present" when both the
`OffboardControlMode` heartbeat *and* an actual `TrajectorySetpoint` stream
are flowing — a node that waits to publish setpoints until after arming will
never be allowed to arm. Additionally, the first offboard-mode/arm command
right after boot is often rejected while preflight checks settle.
`offboard_control_node` therefore streams the takeoff setpoint from tick 0
(harmless while disarmed on the ground) and retries mode-switch + arm once
per second until `vehicle_status` confirms — never fire-and-forget.

<a id="issue-8"></a>

**8. Arming is rejected for the first ~30–60 s after `make sim` — this is
normal.** PX4's EKF2 estimator needs time to converge after boot; until then
the log shows `Preflight Fail: ekf2 missing data` and
`pre_flight_checks_pass: false`. Do not debug this — just wait.
`make flight-test` handles it automatically (the node retries once per
second until PX4 accepts). To watch convergence yourself:
`make shell` → `ros2 topic echo /fmu/out/vehicle_status_v1 --once` and wait
for `pre_flight_checks_pass: true`.

<a id="issue-9"></a>

**9. On a weak/older CPU the simulation runs slower than wall-clock.** PX4
SITL runs in lockstep with Gazebo: when the CPU can't keep up, simulated
time simply advances slower than real time — sim/wall-clock speed is not
fixed and varies run to run (a 2 m climb has taken anywhere from ~13 s to
~77 s on the same dev laptop across sessions). Nothing is hung; the flight is
proceeding correctly in sim-time. Judge progress by the node's state
transitions (climbing → hovering → landing), never by a stopwatch, and be
generous with any `timeout` you wrap around the test.

<a id="issue-10"></a>

**10. If the first `make build` dies mid-way (network hiccup), just rerun
it.** The px4-sim image clones PX4 + all submodules and downloads Gazebo
Harmonic packages — several GB total. Docker caches each completed layer,
so a rerun resumes from the last finished step instead of starting over.

<a id="issue-11"></a>

**11. Both containers use host networking — check for conflicts on shared
machines.** The Micro-XRCE-DDS-Agent binds UDP `8888` on the host; PX4's
MAVLink uses `14550`/`14540`. If another process holds those ports, the
bridge silently fails. Likewise, ROS 2 traffic uses the default
`ROS_DOMAIN_ID=0` — if other ROS 2 systems run on the same host/LAN, their
topics will cross-talk; set a unique `ROS_DOMAIN_ID` for this stack (add it
to the `environment:` of `ros2-autonomy` in `docker-compose.yml`) if that
applies to you.

<a id="issue-12"></a>

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

<a id="issue-13"></a>

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

<a id="issue-14"></a>

**14. PX4's uXRCE-DDS bridge cannot set PX4 parameters at all.** Every other
command in this repo talks to PX4 over its ROS 2 / uXRCE-DDS bridge, but
that bridge carries no `Parameter*` topic in this pinned version (confirmed
by exhaustively grepping PX4's `dds_topics.yaml`) — so switching PX4's
`EKF2_GPS_CTRL`/`EKF2_EV_CTRL` needed a separate MAVLink `PARAM_SET`
side-channel (`common_perception`'s `set_localization_source`). See [§8](reference.md#sec-8) and
`resource/phase3-gps-denied-localization-source.md` for the full mechanism.

<a id="issue-15"></a>

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

<a id="issue-16"></a>

**16. A `RUN ... && rm -rf /var/lib/apt/lists/*` layer wipes the apt cache
for every subsequent `RUN` layer, not just that one.** A later layer's
`apt-get install` (triggered internally by `rosdep install` when building
OpenVINS) failed with `Unable to locate package ros-humble-cv-bridge`
because an earlier layer's cleanup had already removed the package index,
and `rosdep update` (rosdep's own dependency-key database) does **not**
imply `apt-get update` (apt's package index) — two unrelated caches. Fixed
by adding an explicit `apt-get update` at the start of that later layer.

<a id="issue-17"></a>

**17. Real VIO's cruise speed matters more than it looks — too slow starves
scale/bias observability.** `square`/`survey` originally flew at
`max_velocity_m_s: 0.2` under `VIO_BACKEND=openvins`. A landing-phase
divergence ([issue 21](#issue-21)) was traced, in part, by grepping OpenVINS's own debug
log across the whole flight: accelerometer bias was still actively
*growing*, never converged, by landing time (`-0.0066 → -0.0318 →
-0.0174,0.0428`) — vs. converged and stable in an earlier flight at
`1.0 m/s`. Monocular VIO can only observe scale/IMU bias from real
acceleration events; a long, near-constant-velocity flight starves it of
exactly that for the whole mission. Fixed by raising missions to `0.8 m/s`
([§7](reference.md#sec-7)). Don't set this back below ~0.5 for `openvins` flights.

<a id="issue-18"></a>

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
`--source vision` ([§8](reference.md#sec-8)).

<a id="issue-19"></a>

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

<a id="issue-20"></a>

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
([issue 23](#issue-23)) plus the simulated camera sensors are genuine rendering load on
an older host: a real-time factor well under 1.0 (`gz topic -e -t
/world/<world>/stats`) is expected on modest hardware ([issue 9](#issue-9)), and the
slower the host, the more likely the GUI's initial sync loses this race —
so expect to need `make gz-resync` occasionally on such machines.

<a id="issue-21"></a>

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
([issue 17](#issue-17)) look "stationary" to that kind of threshold for their whole
flight. **Fixed one level up instead**, in `common_control` — see
`land_disarm_low_throttle_dwell_s`/`land_disarm_max_timeout_s` ([§7](reference.md#sec-7)): disarm
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

<a id="issue-22"></a>

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

<a id="issue-23"></a>

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

<a id="issue-24"></a>

**24. A `square` mission flew out of the fenced/textured area at speed and
hit the ground — a real crash, reported 2026-07-09, not reproduced on the
very next clean run (this stack's VIO divergence is stochastic, not
deterministic — see [issue 22](#issue-22); a config can run clean several times then
diverge with nothing changed).** Rather than keep chasing a specific
trigger, added a defense-in-depth geofence in `offboard_control_node.py`
independent of root cause: bounds are auto-derived from the current route's
own bounding box (origin + every queued waypoint) plus
`geofence_margin_m`/`geofence_height_margin_m` ([§7](reference.md#sec-7)) — "modular" per the
original ask, meaning it automatically follows whatever a mission's own
`side_length_m`/`area_length_m`+`area_width_m`/etc. describe, with no
mission-type-specific code. A breach in any flying state aborts immediately
to `LAND` — same AUTO_LAND path a normal mission end uses, backed by the
existing estimate-independent disarm fallback ([issue 21](#issue-21)). This can't
distinguish a genuinely-diverged estimate from a real fence-crossing (both
read the same way from the only position source this architecture has) —
that's an accepted tradeoff: a false-positive early landing is a much
cheaper failure than the crash it replaces.

<a id="issue-25"></a>

**25. While reproducing [issue 24](#issue-24), found a second, worse bug: a
flight/mission node's OS process can outlive its own "mission complete" —
`rclpy.shutdown()` doesn't reliably unblock `rclpy.spin()` here (this
project's `rmw_cyclonedds_cpp` RMW leaves non-daemon background threads a
plain interpreter exit won't wait past).** Confirmed live: a leftover
`square_mission` process kept running for minutes after logging "Landed and
disarmed — mission complete," and the OpenVINS instance sharing its launch
— with nothing left to sanity-check it against — ran its own position
estimate away to 100+ meters, distance-traveled 150 m, well past this
`README`'s existing "estimate can drift post-landing" caveat ([issue 21](#issue-21)).
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

<a id="issue-26"></a>

**26. Manually launching an image viewer (`rqt_image_view`) per mission is
a sequencing race, not just extra typing — a viewer started after the
mission has already begun (or, worse, already finished — see [issue 25](#issue-25)'s
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

<a id="issue-27"></a>

**27. `px4_msgs`' generated Python fields (e.g. `VehicleOdometry.position`,
`.q`) are numpy `float32` arrays, not plain Python floats — assigning one
straight into a `geometry_msgs` field (e.g.
`TransformStamped.transform.translation.x`) throws `AssertionError: The
'x' field must be of type 'float'` at publish time, not at import/lint
time.** Hit live while adding `state_tf_publisher` ([§4.3](setup-guide.md#sec-4-3), [§6](reference.md#sec-6)) — the node's
own frame-conversion math (reusing `frame_transforms`, unchanged from the
VIO bridges) was correct, but its output stayed numpy-typed all the way
through since none of the arithmetic in between forces a cast. Fixed by
explicitly `float(v)`-casting every element of `msg.position`/`msg.q`
before conversion. Worth knowing if writing a new node that reads any
`px4_msgs` array field and republishes into a non-PX4 message type
(`px4_msgs`-to-`px4_msgs` republishing, like `loopback_odometry_bridge`,
never hits this — those fields accept numpy `float32` fine).
`state_tf_publisher` (`common_perception`) broadcasts the vehicle's live
TF (`odom` → `base_link`) and an accumulated `/drone/path`, sourced from
`/fmu/out/vehicle_odometry` — the same "up from the start", mission-
lifecycle-independent pattern as [issue 26](#issue-26)'s `rqt-viewer`, run by a fourth
`make sim-gui` container, `rviz2` (`common_perception/launch/viz.launch.py`).
Verified live: `/tf` and `/drone/path` both populate correctly from PX4
boot (no mission needed), RViz2 renders with real GPU OpenGL (not
software fallback), and a full hover flight-test updated both throughout
takeoff/hover/land with zero errors in any of the three GUI containers.

<a id="issue-28"></a>

**28. `VIO_BACKEND=openvins` flights were genuinely inconsistent flight to
flight — accurate on one `square` mission run, drifting or hitting the
`vio_test.sdf` fence on the next, nothing else changed — reported after
Phase 3 Milestone B and analyzed in `resource/Vio_Drift_analysis.txt`.**
Three real, independently-confirmed contributors, all fixed the same
session (2026-07-10):

- **EKF2 trusting a swinging, per-frame-optimistic vision covariance.**
  Confirmed via PX4's own `params_external_vision.yaml`:
  `EKF2_EV_NOISE_MD` defaults to `0` — "measurement noise is taken from
  the vision message and the EV noise parameters are used as a lower
  bound," NOT a fixed trust level — while `openvins_odometry_bridge.py`
  forwards OpenVINS's own raw per-message covariance untouched. So EKF2's
  actual moment-to-moment trust in vision swung with however confident (or
  not) OpenVINS's own estimate happened to feel on a given frame — a
  structural source of run-to-run inconsistency, not a tuning slip.
  `set_localization_source.py` now raises the FLOOR — `EKF2_EVP_NOISE=0.3`/
  `EKF2_EVV_NOISE=0.15` (up from PX4's own `0.1`/`0.1`) when switching to
  `vision` ([§8](reference.md#sec-8)) — starting values, not derived from first principles,
  retune from live results. **First attempt, tried and reverted the same
  session:** `EKF2_EV_NOISE_MD=1` ("ignore the message's covariance
  entirely, use only fixed params") looked like the more thorough fix, but
  two clean, fully-restarted (`docker compose down`/`up`, ruling out a
  stale-container false alarm) flight attempts both stuck at
  `arming_state=STANDBY` the entire test duration, PX4 repeatedly logging
  "Arming denied: Resolve system health failures first" / "Preflight
  Fail: ekf2 missing data". A third attempt, identical except reverting
  only `EKF2_EV_NOISE_MD` back to `0`, armed, flew, and disarmed cleanly,
  so it was reverted. **Caveat found testing the reverted (floor-only)
  config itself: the exact same never-arms symptom occurred once more,
  on an otherwise-identical clean attempt, WITHOUT `EKF2_EV_NOISE_MD`
  touched at all** — 3 of 4 total clean attempts on the final config
  armed/flew/landed normally, 1 didn't. So `EKF2_EV_NOISE_MD=1` probably
  wasn't uniquely responsible for the original failures either — there's
  a separate, pre-existing, NOT-yet-root-caused flakiness in how
  reliably EKF2/OpenVINS reach "estimator initialized" before arming at
  all, independent of any param in this file. Given the original report
  was inconsistent flight behavior across runs, this is plausibly the
  more fundamental issue and the better next investigation target —
  genuinely unresolved as of this entry, not silently patched over.
- **Periodic checkerboard ground texture — a real aliasing trap.**
  `vio_test.sdf`'s ground-level tile grid (added in the Milestone B
  landing-divergence fix, issue-list context above) alternated exactly 2
  shades in a strict checkerboard — every tile corner was locally
  indistinguishable from every other, letting the KLT tracker/RANSAC
  plausibly associate a feature with the WRONG grid corner as the vehicle
  moved. Fixed by re-coloring the same 64 tiles (same positions/geometry,
  zero missing-asset risk — still plain colored boxes, no texture file)
  with a fixed, deterministic hash of each tile's grid indices across an
  8-color palette instead of a 2-color alternation — no periodic pattern
  left for a tracker to alias against, but still fully reproducible (not
  literally randomized) since it's a checked-in world file.
- **RViz2's `Image` display was a separate, UI-only bug, not a flight-
  safety one** (OpenVINS subscribes to the camera topic directly — RViz
  lag never touched the estimator) — see [issue 26](#issue-26)'s `rqt-viewer` for the
  camera feed's own DDS-bandwidth math corrected below. Root cause:
  RViz's `Image` plugin uploads a fresh GPU texture on the same render
  thread as the 3D view, and was a SECOND subscriber to the same raw
  ~83 MB/s stream (1280×720 RGB8 @ 30Hz — `Vio_Drift_analysis.txt`'s own
  estimate of ~27.6 MB/s used 1 byte/pixel instead of RGB8's 3; the real
  number is roughly 3x that, if anything strengthening its conclusion).
  Fixed by dropping RViz's `Image` display entirely (`quad.rviz`) —
  `rviz2` is now TF/path only, `rqt-viewer` remains the camera viewer
  ([§4.3](setup-guide.md#sec-4-3), [§6](reference.md#sec-6)).

None of these three are proven to be the ONLY causes of the reported
instability — re-test (`make mission MISSION=square
LOCALIZATION=vision VIO_BACKEND=openvins`) multiple times, the same
"looks right, verify live" discipline every VIO fix in this project has
needed. `vio_test.sdf`/the d435i model are baked into the `px4-sim` image
(`docker/Dockerfile.px4_sim` `COPY`s them at build time, not bind-mounted)
— `make build` (or `docker compose --profile sim build px4-sim`) is
required for the re-colored ground tiles to actually take effect, not
just `make build-ws`.

<a id="issue-29"></a>

**29. Arming-reliability gap found verifying the 3 fixes above, then fully
ROOT-CAUSED the same day — a SITL performance issue, not a code bug in
this project, PX4, or OpenVINS.** Across 4 clean, fully-restarted flight
attempts on the final (safe) config from [issue 28](#issue-28), 3 armed/flew/landed
the full `square` mission normally and 1 never armed in 260s, stuck on
PX4's own "Preflight Fail: ekf2 missing data" / "waiting for estimator to
initialize" (`EstimatorCheck.cpp`) — the identical symptom seen (and
initially, incorrectly, attributed solely to `EKF2_EV_NOISE_MD=1`) before
that param was reverted in [issue 28](#issue-28), proving it wasn't uniquely
responsible either.

**A user-submitted analysis doc (`resource/Vio_Drift_analysis.txt`)
proposed a wall-clock-vs-simulation-time ROS node clock mismatch
(`use_sim_time`) as the cause. Investigated and rejected**, with evidence:
`loopback_odometry_bridge.py` (Milestone A, reliable throughout this
project's history) stamps `VehicleOdometry` with the exact same
`self.get_clock().now()` (wall-clock) pattern `openvins_odometry_bridge.py`
uses — if wall-clock stamping broke PX4/EKF2 fusion this badly, Milestone
A would already be broken too. `docker-compose.gui.yml`'s `ros_gz_bridge`
config bridges no `/clock` topic at all, so blindly setting
`use_sim_time:=true` as proposed would have starved every node's ROS
clock of a source — a real risk of making things worse, not better.

**Actual root cause, traced through OpenVINS's own source and confirmed
live**: `ROS2Visualizer::visualize_odometry()` (`ov_msckf`, not this
project's code) unconditionally returns without publishing ANYTHING until
`(timestamp - _app->initialized_time()) >= 1` — one full second of
OpenVINS's own internal, Gazebo-simulation-derived time since
initialization. Confirmed live during a stuck attempt:
`gz topic -e -t /world/vio_test/stats` measured `real_time_factor: 0.023`
(~1/44th of real time) — `gz sim` alone was consuming 448% CPU on an
8-core host, load average >5. At that RTF, OpenVINS's 1-second (sim-time)
gate needs ~44 REAL seconds just to start publishing, confirmed via
`ros2 topic hz /fmu/in/vehicle_visual_odometry` showing ZERO messages
across a full 250-second wall-clock window during a stuck run — easily
exceeding any reasonable arm-retry patience, with nothing actually broken
underneath (OpenVINS was healthy, still running ZUPT updates the whole
time; the DDS graph showed the publisher and this project's subscriber
correctly discovered with fully compatible QoS — the block is entirely
inside OpenVINS's own gate, not a discovery race).

**This is 100% specific to SITL** (Gazebo's lockstep simulated clock,
confirmed via PX4 subscribing to `/world/<world>/clock` in
`px4-rc.gzsim`) **and cannot recur on Phase 4 real hardware**, which has
no simulated clock at all — OpenVINS there processes real camera frames
in real wall-clock time. Given the original report was inconsistent
flight-to-flight behavior, RTF variance under CPU load (worse when the
host is already busy — e.g. from a long dev session with many
`docker compose` restarts, as this one was) plausibly explains a real
share of it directly, independent of the 3 fixes in [issue 28](#issue-28).

No code fix applied — this needs mitigation, not a patch, since the
underlying mechanism (OpenVINS's 1-second gate) isn't this project's code
to change ([§11](../README.md#no-upstream-repos-are-forked-or-modified), no upstream forks). Options, not yet applied, roughly
priority order:
- Reduce simulation computational load: fewer/lighter `vio_test.sdf`
  props, lower camera resolution/framerate (also serves [issue 28](#issue-28)'s RViz
  bandwidth fix — would need re-deriving `kalibr_imucam_chain.yaml`'s
  intrinsics for the new resolution, a real recalibration, not just a
  config edit), or reduce OpenVINS's `track_frequency`/`num_pts` if still
  bottlenecked after that.
- Test on a genuinely idle host — this session's 0.023 RTF was measured
  after hours of continuous `docker compose` restarts/exec calls; a fresh
  baseline (reboot, nothing else running) may be substantially better.
- Don't assume "stuck" from a short timeout — check
  `gz topic -e -t /world/<world>/stats`'s `real_time_factor` directly
  before concluding something is broken; this is slowness, not a hang.

Real stereo VIO (feeding the D435i's actual left/right IR pair instead of
a single mono color camera, giving OpenVINS a static metric baseline
instead of needing IMU excitation to observe scale at all) remains the
next, larger step under consideration for the drift/scale side of
instability — not yet implemented as of this entry. It would inherit this
same RTF-dependent arming gap unchanged (OpenVINS's 1-second rule isn't
mode-specific), so testing on a healthier-RTF host is worth doing before
or alongside adding stereo, not strictly required first.

<a id="issue-31"></a>

**31. `/dev/dri` (GPU rendering) backported to the headless base
`docker-compose.yml` (2026-07-13) — turns out it wasn't just a vision-mode
speed issue.** User reported GPS-only flights (no camera, no VIO) also
drifting and false-tripping the geofence right after arming, and a square
mission that visually looked like it never completed. Root cause:
identical to [issue 29](#issue-29)/30's RTF finding, but hitting the GPS/EKF2-settle
path this time — without `/dev/dri`, the headless container
software-rendered its cameras on CPU (RTF ~0.02-0.03 even for a mission
using no camera data at all, since Gazebo still renders every sensor every
frame regardless of who's subscribed), and this README's own "PX4's EKF2
needs ~30-60s after `make sim`" guidance is REAL-time advice that assumes
RTF≈1 — at RTF 0.02, 30-60 real seconds only buys EKF2 ~1-2 *simulated*
seconds, nowhere near enough to converge, while PX4's arm-gate is looser
than "fully converged" and lets the mission arm anyway. Confirmed live: a
GPS-only hover test false-tripped its own geofence on the very first tick
after arming without `/dev/dri`; with it (and a real ~35s wait), the same
test hovered cleanly and a `square` mission hit all 5 waypoints in order
and landed within ~2m of center. `/dev/dri` is now baked into the base
compose file (previously only the GUI overlay had it) — mandatory for
**both** `LOCALIZATION=gps` and `LOCALIZATION=vision` reliability, not
just the sim-gui/VIO path. **Portability**: this makes GPU render nodes
required to start the stack at all — on a GPU-less VM/server, `docker
compose up` fails with a device error; comment the `devices:` block out
there.

<a id="issue-32"></a>

**32. `/drone/path` (RViz) reset on arm (2026-07-13)** — `state_tf_publisher`
never restarts between flights by design ([§4.3](setup-guide.md#sec-4-3)), so two unrelated flights
run back to back used to draw as ONE continuous connected line — the last
pose of flight A joined straight to the first pose of flight B. A user
screenshot of exactly this (two flights' paths tangled into one shape via
a straight connecting line) was initially, reasonably mistaken for a
single badly-drifting square mission. Fixed: `state_tf_publisher` now
subscribes to `vehicle_status_v1` and clears its path buffer on every
disarmed→armed transition, so each flight draws its own trace. Confirmed
live across two consecutive flights (a hover then a `square` mission) —
the reset fired exactly once per arm.

<a id="issue-33"></a>

**33. `FlightState.LAND` had NO geofence check at all — confirmed as the
direct cause of a real vehicle flying into the fenced area's wall during
landing (2026-07-13).** `_check_geofence()` was called from
`TAKEOFF`/`WAYPOINTS`/`HOVER` but never from `LAND` — once a mission
reached "all waypoints reached — landing," PX4's own `AUTO_LAND`
controller took over the descent with zero further oversight from this
project's code. This project already has a documented prior incident of
`AUTO_LAND` flying away when its position estimate diverges during
descent ([§8](reference.md#sec-8), the extrinsics saga); with no geofence watching that phase,
nothing stopped it from flying the real vehicle into a wall — reported by
the user across repeated vision-mode test flights, one of which hit a
fence boundary specifically during landing.

Two-part fix in `offboard_control_node.py`:
- **`geofence_hard_limit_m`** — a NEW required param, an absolute x/y
  clamp on `_geofence_bounds()`'s output, independent of any mission's own
  waypoint-bbox-derived margin. Whichever of (mission bbox + margin) or
  (hard limit) is more restrictive wins, so a generously-margined mission
  can never accidentally widen the fence past a known-safe physical
  limit. Set to `3.75` in `sim_params.yaml` (`vio_test.sdf`'s fence props
  sit at ±4.75m — this guarantees ≥1m wall clearance regardless of
  mission geometry); `10.0` as an explicitly-flagged UNTESTED placeholder
  in `hw_params.yaml` (no fixed test-area geometry to derive it from yet
  — must be retuned to the real test area before free flight).
- **`_check_geofence(during_land=True)`**, now called from
  `FlightState.LAND` too. Its response is deliberately DIFFERENT from the
  normal path: `_land()` is a no-op there (`AUTO_LAND` is already the
  active mode — re-commanding it changes nothing), and there's no way to
  hand it a corrected trajectory from here, so this is a genuine last
  resort — **force-disarm immediately** (`_disarm()`, PX4's documented
  force-disarm magic value, already used by the existing post-landing
  fallback). At this project's low mission altitudes (2m) an uncontrolled
  drop is short and, on the evidence available, safer than continuing to
  fly toward/through a wall. Ends the flight (`FlightState.DONE`) rather
  than attempting recovery — a human should inspect the vehicle after
  this fires.

**A real bug was caught and fixed while building this**: the two response
paths originally shared one latch flag (`_geofence_breached`), which meant
a mid-cruise breach (setting the flag, transitioning normally to LAND)
would silently BLOCK the during-land emergency check from ever firing if
`AUTO_LAND` then diverged further during the resulting descent — exactly
the compounding failure this fix exists to catch. Fixed with two
independent latches (`_geofence_breached` / `_geofence_land_emergency`).
Verified live: teleporting the vehicle out of bounds mid-flight (Gazebo's
own `set_pose` service) triggered the normal breach-and-land response,
and on the VERY NEXT control tick (100ms later) the during-land check
independently caught the still-out-of-bounds position and force-disarmed
— confirming the compounding scenario the latch fix targets actually
works end to end, not just in isolation. A normal, uninterrupted `square`
mission was re-verified clean after every change in this fix (no
false-positive triggers from the new hard limit at this mission's normal
geometry).

**Still open**: this closes a real safety gap, but does not fix VIO
accuracy/divergence itself ([issues 29-32](known-issues.md#issue-29)) — it makes the WORST case
(an undetected divergence during landing) survivable instead of
catastrophic. See [issue 34](#issue-34) for the broader in-flight watchdog built the
same day.

<a id="issue-34"></a>

**34. In-flight estimate-health watchdog — built the same day [issue 33](#issue-33)
was fixed, after a concrete, live crash proved it necessary.** A VIO
pipeline node (`openvins_odometry_bridge`) crashed outright mid-flight —
`RuntimeError: Unable to convert call argument to Python object`, raised
INSIDE `rclpy`'s own message-deserialization step
(`executors.py`'s `sub.handle.take_message(...)`, before the node's own
callback is ever reached) — silently stopping all vision data to PX4 for
the rest of that flight, with nothing watching for it. Traced to severe
host CPU contention (load average 15+ measured live, `rviz2`'s GPU
rendering alone at 60%+ CPU) — a known category of rclpy/CycloneDDS
fragility under resource starvation, not a logic bug in the bridge's own
code. That flight happened to still land only mildly drifted; nothing
would have caught a worse outcome.

Two complementary fixes:
- **Bridge resilience** (`openvins_odometry_bridge.py`): replaced plain
  `rclpy.spin(node)` (lets any uncaught exception kill the whole process)
  with a manual `spin_once` loop that catches `RuntimeError` and keeps
  going — survives a single transient DDS hiccup instead of dying
  outright. Doesn't fix the underlying rclpy/DDS fragility (host
  contention is the real trigger) and only protects this one node — see
  the next fix for the general case.
- **Estimate-health watchdog** (`offboard_control_node.py`, new
  `_check_estimate_health`) — the general, source-agnostic version:
  reads PX4's OWN `VehicleLocalPosition.xy_valid`/`v_xy_valid` — its
  internal judgement of whether the current fused estimate is
  trustworthy, already true regardless of which aiding source produced
  it. Deliberately NOT vision-specific (doesn't watch
  `/fmu/in/vehicle_visual_odometry` or any vision topic directly) —
  keeps this class unaware of whether GPS or vision is active, the same
  invariant `common_control`/`common_missions` maintain everywhere else
  in this project. Fires on ANY sustained estimate-health problem — a
  crashed bridge, a wedged DDS agent, or genuine divergence bad enough
  that PX4 itself stops trusting the fusion all trip the same flag.
  Same TAKEOFF/WAYPOINTS/HOVER/LAND coverage and the same
  during-land-force-disarms-as-last-resort shape as [issue 33](#issue-33)'s geofence
  (including its own independent two-latch pair, same compounding-breach
  reasoning). One difference: a short dwell
  (`estimate_invalid_abort_dwell_s`, `1.5s` sim / `2.0s` hw) before
  aborting — a single-tick validity flicker shouldn't end a flight by
  itself the way a hard position-bound breach should.

**A real, separate bug was found and fixed while wiring the new param**:
`hw_params.yaml`'s `square_mission` section was missing
`geofence_hard_limit_m` entirely (added in [issue 33](#issue-33), only added to the
hover profile's section by mistake) — would have failed loudly at
startup (this project's params have no silent code defaults) rather than
silently misbehaving, but still a real oversight, now fixed.

**Verified live, twice, for two different things**:
- Combined test: killed GPS fusion via MAVLink mid-`square`-mission
  (directly simulating "GPS is not there," the user's other question)
  with vision not running either — dead-reckoning drift reached the
  geofence bound BEFORE the estimate-health dwell elapsed; geofence
  caught it, transitioned to LAND, and the during-land emergency check
  force-disarmed on the next tick. Both safety layers worked together.
- Isolated test: killed GPS mid-`hover` (vehicle not commanded to move,
  so drift stays small and slow) with an extended test-only hover
  duration to give the watchdog room to react without the mission just
  landing normally first. `xy_valid`/`v_xy_valid` went false ~6.7s after
  the kill (matching PX4's own 5s dead-reckoning timeout + this fix's
  1.5s dwell), the normal path aborted to LAND, and the independent
  during-land check fired on the very next control tick and
  force-disarmed — a clean, geofence-free demonstration of this fix
  specifically, not just the two working together.

<a id="issue-35"></a>

**35. `VIO_BACKEND=openvins` silently starved of all camera/IMU data when
`PX4_GZ_WORLD=vio_test` is forgotten — looked like "stuck waiting for
arm/offboard forever," root cause was one launch-time env var
(2026-07-14).** User report: `square_mission` logs showed
`nav_state=14`/`arming_state=1` (offboard engaged, never armed) repeating
`Arm command sent` / `Waiting for offboard+armed` indefinitely. Live
repro found TWO symptom variants of the identical root cause, both
reproduced this session: sometimes PX4 arms almost instantly on
residual pre-switch EKF2 state and then [issue 34](#issue-34)'s estimate-health
watchdog aborts-to-LAND ~4s into the flight; other times arming itself
never succeeds. Which variant appears is just timing (how much stale
GPS-based validity EKF2 has left at the exact instant `set_
localization_source` flips `EKF2_GPS_CTRL`/`EKF2_EV_CTRL`) — not two
different bugs.

**Actual root cause**: `ros_gz_bridge`'s config
(`config/ros_gz_bridge.yaml`) subscribes to Gazebo topics under
`/world/vio_test/model/x500_depth_0/...` — these only exist when the sim
was started with `PX4_GZ_WORLD=vio_test` ([§4.3](setup-guide.md#sec-4-3)/[§8](reference.md#sec-8)). `make sim`/`make
sim-gui` default to `PX4_GZ_WORLD=empty` (documented, and correct for
GPS-mode flights — no reason to pay `vio_test`'s extra prop/tile
rendering cost when not testing vision). Start the sim on `empty` and
then fly `VIO_BACKEND=openvins` anyway, and `camera_imu_bridge` still
starts cleanly and creates all the right ROS topics — they just never
receive a single message, because Gazebo is publishing under
`/world/empty/...` instead. OpenVINS then sits forever with zero camera/
IMU input, logs nothing (confirmed live: zero log lines from
`run_subscribe_msckf` for the whole flight), and the position estimate
never becomes valid. Nothing upstream can catch this: `set_
localization_source` only talks to PX4 over MAVLink and has no
visibility into Gazebo's topic tree; `offboard_control_node` retrying
arm/offboard once a second forever is *correct* behavior for the normal
30-60s EKF2-convergence case ([issue 8](#issue-8)) — from the mission log alone,
"no vision data will ever arrive" is indistinguishable from "still
converging, give it time."

**Fix — fail LOUD instead of silently hanging**: `vio.launch.py` gained
a fourth concurrent action, `camera_data_check` — waits up to 8s for one
message on `/camera/camera/imu` (`timeout 8 ros2 topic echo ...
--once`) and, if none arrives, logs an unmissable `[camera_data_check]
ERROR` naming the exact cause and the fix (`PX4_GZ_WORLD=vio_test make
sim`). Deliberately doesn't gate or delay the other three actions (VIO
still starts immediately as before) — it only adds a bounded,
actionable diagnostic where previously there was none. **Verified live
both ways**: reran the identical mission twice back to back, same
sim session, only `PX4_GZ_WORLD` changed — `vio_test` flew clean (armed,
climbed, all 5 waypoints, landed, [issue 21](#issue-21)'s post-landing fallback
disarm handled the still-open post-landing-drift gap as designed);
`empty` reproduced the original hang/early-abort *and* printed the new
`camera_data_check` error within 8s pointing straight at the fix.

**Separately noticed, not yet root-caused, does not block flights**:
`run_subscribe_msckf` (OpenVINS's own process) exits with SIGSEGV
(code -11) on every run observed this session — but only during the
launch's own SIGINT/shutdown sequence, after the mission has already
landed or force-disarmed. Looks like a crash in OpenVINS's own cleanup
path, not a flight-time issue — flagged here so a future session
doesn't mistake it for a new regression, but not chased down; it never
appeared to affect an in-progress flight.

**Also note**: `set_localization_source.py` on disk already carries an
`EKF2_NOAID_TOUT` change (vision mode only, PX4's own max of 10s instead
of the 5s default) from earlier the same day, addressing a related but
distinct real-time-budget squeeze once RTF is genuinely 1.0 (see the
module's own docstring). That change is independent of this issue —
confirmed present and applied (`EKF2_NOAID_TOUT = 10000000 (confirmed)`
in the MAVLink switch log) in every repro run this session, vision-data
starvation just meant it never got a chance to matter.

<a id="issue-36"></a>

**36. Follow-up to [issue 35](#issue-35): `vio_test` was still hardcoded into
`ros_gz_bridge.yaml`'s topic paths — fixed properly instead of just
patched around (2026-07-14, same day).** Issue 35's fix made a wrong
`PX4_GZ_WORLD` fail loudly, but didn't address the actual bad practice
the user flagged after reading the diff: the bridge's Gazebo-side topic
paths (`/world/vio_test/model/x500_depth_0/...`) were still a literal
hardcoded string, meaning any future second VIO-capable world would need
a hand-edited YAML, not just a different `PX4_GZ_WORLD` value — the
exact kind of hardcoding that caused [issue 35](#issue-35) in the first place, just
not yet triggered a second way.

**Fix**: `config/ros_gz_bridge.yaml` is now a TEMPLATE
(`__WORLD__`/`__MODEL__` placeholder tokens, not literal paths).
`vio.launch.py` gained `world`/`model` launch arguments — `world`
defaults to the `PX4_GZ_WORLD` environment variable (wired into the
`ros2-autonomy` container via `docker-compose.yml`, which previously
only passed it to `px4-sim`) — and a new `_bridge_config_with_world`
function renders the template with the resolved values into a temp file
before `parameter_bridge` starts (plain string substitution — no
templating engine needed for two tokens; `parameter_bridge`'s
`config_file` param is loaded as static YAML with no substitution
support of its own). Net effect: `PX4_GZ_WORLD=<any-world> make sim`
now "just works" for the bridge automatically, with exactly one place to
set the world — not the config file AND the command line, which could
silently disagree.

**A real gap in [issue 35](#issue-35)'s own guard was found and fixed while verifying
this.** The original `camera_data_check` watched `/camera/camera/imu` —
reasonable when the bug was "topics don't exist at all," but a live
regression test after this fix exposed why that was the wrong signal in
general: once the bridge always matches the running world, IMU data
flows on ANY world, including `empty` — an IMU is a physics sensor,
indifferent to ground texture. So the guard would have silently gone
blind to the *other* already-documented openvins requirement ([issue 8](#issue-8)'s
sibling note: `empty`'s flat untextured ground gives OpenVINS's feature
tracker zero usable corners even with real data flowing) the moment
[issue 35](#issue-35)'s original bug was fixed — replacing one silent failure mode
with a different, newly-silent one. Renamed to `vio_output_check` and
repointed at OpenVINS's own output (`/ov_msckf/odomimu`, 15s timeout —
longer than the 8s before, since this now has to tolerate a legitimately
slower-but-fine static init rather than just detect zero-vs-nonzero
input) — checking the output catches both causes (wrong world, or a
world with data but not enough texture) with one mechanism, since both
end in the same observable fact: no valid estimate ever comes out.

**Verified live, three ways, same sim session**: `vio_test` with the new
template rendering → full mission (armed, climbed, all 5 waypoints,
landed) with the correctly-rendered `/world/vio_test/model/
x500_depth_0/...` paths confirmed in the bridge's own startup log.
`empty` with the OLD `/camera/camera/imu`-based check → check passed
cleanly (proving the blind-spot theory: IMU data really does flow fine
on `empty`) while the mission still failed the same way as [issue 35](#issue-35)
(`POSITION ESTIMATE INVALID`, force-disarm) — this is what caught the
gap. `empty` again after switching to the `/ov_msckf/odomimu`-based
check → guard now correctly fires within 15s, naming both possible
causes. Re-verified `vio_test` a final time against the corrected check
to confirm no false-positive on the good path.

<a id="issue-37"></a>

**37. `run_subscribe_msckf` (OpenVINS's own process) segfaults on every
shutdown — root-caused to upstream `rclcpp`, confirmed benign
(2026-07-14).** Flagged as an open question in issues 35/36 (`exit code
-11` on every run, only during the launch's own SIGINT teardown). User
asked for an actual analysis rather than leaving it as a shrug. Read
OpenVINS's own source (`/opt/openvins_ws/src/open_vins/ov_msckf/src/
run_subscribe_msckf.cpp` inside the container — vendored upstream
`rpng/open_vins`, not code in this repo): `main()` runs an
`rclcpp::executors::MultiThreadedExecutor` (spins OpenVINS's camera/IMU
callbacks across several worker threads), and the instant `spin()`
returns (triggered by SIGINT), immediately calls `viz->
visualize_final()` — which reads the shared `VioManager`/
`ROS2Visualizer` state (camera intrinsics, timeoffset, extrinsics) — and
then `rclcpp::shutdown()`. This is a known, still-open category of
`rclcpp` fragility: `MultiThreadedExecutor::spin()` returning on the
calling thread does not always mean every worker thread it spawned has
actually finished touching node state first, so a callback still
in-flight on another thread can race the main thread's post-spin
teardown — a classic concurrent-access crash shape, not unique to
OpenVINS (see e.g. upstream `ros2/rclcpp`/`ros2/launch` issues on
SIGINT-vs-executor-thread races).

**Confirmed against the user's own crash log, not just theory**: the
two lines immediately preceding every crash —
```
camera-imu timeoffset = 0.01667
cam0 intrinsics = 465.922,466.035,320.933,179.928 | 0.001,0.000,0.004,0.002
```
are `visualize_final()`'s own `PRINT_INFO` output, character-for-character
matching its source. The crash happens during or immediately after this
exact function, exactly where the race theory predicts.

**Verdict: benign, upstream, not worth patching.** Confirmed across
every reproduction this project has hit it (7+ runs, one session) that
it ONLY happens after `Landed and disarmed — mission complete` — never
mid-flight, never before disarm. The fragility is in `rclcpp`'s own
executor/signal-handling internals (an area ROS 2's own maintainers have
open, unresolved issues about), not something a few lines of local
patching to vendored third-party source would reliably fix — and this
project doesn't fork/patch upstream repos by policy. No action taken;
documented so no future session re-investigates this from scratch or
mistakes it for a new regression.

---

[← Back to README](../README.md) · [Technical Reference](reference.md)
