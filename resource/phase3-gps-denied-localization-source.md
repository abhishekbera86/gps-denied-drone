# Phase 3 — GPS-Denied Flight with a Chosen Localization Source (VIO or GPS)

> Design record for Phase 3, written before implementation began. Kept as a
> durable, versioned reference — unlike `IMPLEMENTATION_PLAN.md` and
> `DEVELOPMENT_STATUS.md` (gitignored, fast-changing working docs), this file
> is meant to survive: the *why* behind the localization-switch design,
> including dead ends ruled out, so Phase 4 hardware work doesn't have to
> re-derive this research.

## Objective

Let the operator choose the drone's localization source **before a mission
starts**: fly outdoors → GPS; fly indoors → the Intel RealSense D435i camera
drives Visual-Inertial Odometry (VIO) instead. This is a deliberate,
launch-time choice, not a live in-flight failover switch (that is future
work, intentionally deferred — see Non-Goals).

**Guiding principle:** robotics best practices, simple maintainability, and
an easy sim→real translation path. This is the same Rule already governing
this repo's architecture (`common_autonomy` — `common_control`,
`common_missions`, `common_perception` — is shared and unmodified between
sim and real hardware; `sim_bringup`/`hw_bringup` differ only in connection
config) applied to localization specifically, not a new principle invented
for this feature.

---

## Key findings from PX4/ROS 2 source research

1. **PX4's uXRCE-DDS bridge cannot set PX4 parameters at all**, at runtime or
   otherwise. Exhaustive search of PX4 v1.17.0's `dds_topics.yaml` (the file
   that decides what's bridged) found zero `Parameter*` topics; the generic
   MAVLink `VEHICLE_CMD_DO_SET_PARAMETER` (180) exists in the message
   definitions but is unhandled by PX4's own `Commander.cpp`. The only way
   to change `EKF2_GPS_CTRL`/`EKF2_EV_CTRL` from a companion computer is a
   **MAVLink `PARAM_SET` side-channel** (via `pymavlink`) — PX4 SITL already
   exposes a MAVLink UDP port independently of the DDS agent, so this needed
   no PX4 rebuild or fork. Confirmed from source: **neither
   `EKF2_GPS_CTRL` nor `EKF2_EV_CTRL` requires a reboot** to take effect (no
   `reboot_required` flag on either) — they apply on the next EKF2
   parameter-update tick.

2. **`EKF2_HGT_REF` (which height source is the reference) DOES require a
   reboot.** Rather than fight that, this design uses PX4's own automatic
   height-source fallback (`checkHeightSensorRefFallback()`, confirmed in
   source, runs every EKF2 cycle): leave `EKF2_HGT_REF` untouched at its
   default (GPS), and when `EKF2_GPS_CTRL=0`, GPS simply never becomes
   "currently fusing," so PX4 automatically falls back to the next enabled
   height source — **baro** (on by default, always available, needs no GPS
   or camera). So the switch only ever needs to touch **`EKF2_GPS_CTRL`**
   and **`EKF2_EV_CTRL`** — never `EKF2_HGT_REF`, sidestepping the reboot
   requirement entirely. This is standard practice for indoor PX4 setups
   (baro for Z, vision for XY).

3. **PX4 already exposes everything needed to read the result back** — no
   gap to fill: `/fmu/out/estimator_status_flags` (`EstimatorStatusFlags`,
   DDS-bridged today) carries per-source fusion booleans (`cs_gnss_pos`,
   `cs_ev_pos`, etc.) to confirm which source is actually active.
   `/fmu/in/vehicle_visual_odometry` (`VehicleOdometry`, DDS-bridged,
   unused until this phase) is the topic a vision source publishes to.

4. **`common_control`/`common_missions` never touch odometry source** — they
   only ever read the already-fused `vehicle_local_position_v1`. The entire
   localization-source mechanism lives below/alongside `common_control` with
   **zero changes to the already-working control or mission code**.

5. **Gazebo's stock `x500_depth` model ships a generic "OakD-Lite" sensor**,
   not a D435i simulation — confirmed live via `gz topic -l`: RGB camera
   (`IMX214`), depth camera (`StereoOV7251`, explicit topic `/depth_camera`),
   and a camera-mounted IMU, all under
   `/world/empty/model/x500_depth_0/link/camera_link/sensor/...`. Later
   replaced with our own local `d435i` model (see Milestone B finding 6) so
   sim actually flies the same camera as real hardware, not a stand-in with
   matched topic names.

6. **Real D435i topic names** (`realsense-ros` 4.58.2, the release paired
   with `LIBREALSENSE_VERSION=2.58.2` already pinned in this repo's `.env`
   for Phase 4), confirmed from source — the exact target the Gazebo bridge
   remaps to, so sim and real hardware present as "the same camera":

   | Stream | Topic | Type |
   |---|---|---|
   | Color | `/camera/camera/color/image_raw` | `sensor_msgs/Image` (rgb8) |
   | Color info | `/camera/camera/color/camera_info` | `CameraInfo` |
   | Depth | `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` (16UC1, mm) |
   | Depth info | `/camera/camera/depth/camera_info` | `CameraInfo` |
   | IMU (united) | `/camera/camera/imu` | `sensor_msgs/Imu` (fully populated) |

   The real D435i needs `unite_imu_method:=1|2` to get one populated `imu`
   topic (otherwise gyro/accel publish separately, each half-populated).
   Gazebo's IMU sensor has no such split — it always publishes both fields
   at once, so the **sim side is "pre-united" for free** and bridges
   straight to `/camera/camera/imu`. Namespace is double-nested
   (`/camera/camera/...`, not `/camera/...`) — a `realsense-ros` version
   change worth flagging since older tutorials use the single-level form.

7. **OpenVINS**: no apt package for Humble — build from source
   (`github.com/rpng/open_vins`: `ov_core`/`ov_init`/`ov_msckf`). Its own
   `Dockerfile_ros2_22_04` is a confirmed-working Humble dependency list
   (Eigen, Ceres via apt, OpenCV via `ros-humble-desktop`). It publishes
   `ov_msckf/odomimu` (`nav_msgs/Odometry`); there is no off-the-shelf
   OpenVINS→PX4 bridge anywhere — a small custom node is the norm in every
   example found. No `d435i` OpenVINS example config exists upstream
   (checked live); closest is `config/rs_d455`, which uses old single-level
   topic names in its `rostopic:` fields — those need hand-editing to the
   real double-nested names above regardless, since OpenVINS has no
   separate topic-remap layer (the YAML's `rostopic:` field is
   authoritative).

8. **Frame convention gotcha**: OpenVINS's world frame is gravity-aligned
   but has an *arbitrary, non-North-aligned yaw* at init — so the PX4 bridge
   must set `VehicleOdometry.pose_frame = POSE_FRAME_FRD`, never
   `POSE_FRAME_NED`. The ENU/FLU→NED/FRD axis conversion is simple (axis
   negation, not a full rotation) — implemented directly in Python rather
   than vendoring PX4's C++ `frame_transforms` header, since this workspace
   is pure `ament_python`.

---

## Architecture

### The one-shot localization switch

A plain Python script (console_script `set_localization_source`, no rclpy
`Node`/spin needed — it's a single imperative MAVLink transaction, not a ROS
node), `common_perception/common_perception/set_localization_source.py`:

```
set_localization_source --source gps|vision --mavlink-url udp:127.0.0.1:14540
```

1. Connects via `pymavlink` to the given MAVLink endpoint (SITL exposes one
   on UDP independently of the DDS agent; real hardware's endpoint is TBD
   Phase 4, same "untested stub" status as the rest of `hw_bringup`).
2. Looks up `source` in a small constant dict (the only two entries needed
   now; adding a third source later is one more entry):
   ```python
   SOURCES = {
       "gps":    {"EKF2_GPS_CTRL": 7, "EKF2_EV_CTRL": 0},
       "vision": {"EKF2_GPS_CTRL": 0, "EKF2_EV_CTRL": 5},  # HPOS(1)+VEL(4); height stays on baro
   }
   ```
3. Sends `PARAM_SET` for each, reads back to confirm, logs success/failure,
   exits non-zero on failure — fail loud, same philosophy as
   `OffboardControlNode._require_param`, just not a ROS parameter this time.
4. `sim_bringup`/`hw_bringup` launch files run this as an `ExecuteProcess`,
   sequenced via `RegisterEventHandler(OnProcessExit(...))` so the mission
   node only starts after it exits 0 — a standard ROS 2 launch idiom.
5. New launch args on both bringups: `localization_source` (default `gps`)
   and `mavlink_url` (bringup-specific default, same precedent as
   `hw_bringup`'s existing `serial_device`/`baud` args — connection config
   lives in the bringup layer, not a YAML params file). The `Makefile` gets
   `LOCALIZATION ?= gps`, threaded exactly like the existing `MISSION ?=`
   variable: `make mission MISSION=square LOCALIZATION=vision`.

### VIO odometry → PX4 bridge

A shared module `common_perception/common_perception/frame_transforms.py`
(small, self-contained ENU/FLU→NED/FRD axis + quaternion conversion), used
by two thin bridge nodes that both publish the identical target
topic/type — `common_control` never knows or cares which one is running:

- **`loopback_odometry_bridge.py`** (Milestone A) — subscribes to the
  already-live `/fmu/out/vehicle_odometry` (PX4's own current GPS-fused
  estimate) and republishes it as `/fmu/in/vehicle_visual_odometry`. Zero
  new Gazebo/camera work required — this alone proves the
  switch+EKF2+bridge mechanics end to end, before OpenVINS's own estimation
  error is introduced as a second variable.
- **`openvins_odometry_bridge.py`** (Milestone B) — subscribes to
  `ov_msckf/odomimu` (`nav_msgs/Odometry`), applies `frame_transforms`, sets
  `pose_frame = POSE_FRAME_FRD`, republishes to the same topic.

### Gazebo → ROS 2 camera/IMU bridge (Milestone B)

`common_perception/config/ros_gz_bridge.yaml`, launched via
`ros_gz_bridge`'s `parameter_bridge`, remapping the live Gazebo topics to
the real-D435i topic names (finding 6): `.../sensor/color/image` →
`/camera/camera/color/image_raw`, `.../color/camera_info` →
`/camera/camera/color/camera_info`, `/depth_camera` →
`/camera/camera/depth/image_rect_raw`, `.../sensor/imu/imu` →
`/camera/camera/imu` (our own `d435i` model's own onboard IMU — see
Milestone B finding 6, not the OakD-Lite's dangling `camera_imu` sensor
finding 2 originally worked around). Depth encoding will differ from real
hardware (Gazebo's depth sensor is float meters, D435i is `16UC1` mm) —
flagged but not solved this phase, since OpenVINS's MSCKF doesn't consume
the depth stream for VIO; only color+IMU need to match exactly.

### OpenVINS (Milestone B)

Built from source in `docker/Dockerfile.ros2_autonomy` (new layers appended
after the existing colcon-build layer — editing mission Python never
invalidates this), using OpenVINS's own `Dockerfile_ros2_22_04` as the
dependency reference. Config lives in `common_perception/config/openvins/`
(`estimator_config.yaml`, `kalibr_imu_chain.yaml`,
`kalibr_imucam_chain.yaml`), adapted from OpenVINS's `config/rs_d455`
starting point with `rostopic:` fields hand-edited to the real
double-nested D435i names. No new Docker service/container — everything
runs inside the existing `ros2-autonomy` image, matching
`IMPLEMENTATION_PLAN.md`'s original architecture diagram ("vio: (same
image)").

---

## File layout

```
ros2_ws/src/common_perception/            NEW package (ament_python)
  common_perception/
    set_localization_source.py            one-shot MAVLink PARAM_SET script
    frame_transforms.py                   shared ENU/FLU -> NED/FRD math
    loopback_odometry_bridge.py           Milestone A bridge
    openvins_odometry_bridge.py           Milestone B bridge
  launch/vio.launch.py                    ros_gz_bridge + OpenVINS + Milestone-B bridge
  config/ros_gz_bridge.yaml               gz <-> ROS 2 topic map (Milestone B)
  config/openvins/*.yaml                  estimator + kalibr calibration (Milestone B)
  package.xml / setup.py / setup.cfg      standard package plumbing

ros2_ws/src/sim_bringup/launch/sim.launch.py   + localization_source/mavlink_url args,
ros2_ws/src/hw_bringup/launch/hw.launch.py       switch sequencing, conditional vio.launch.py include
docker/Dockerfile.ros2_autonomy                + pymavlink, ros-gz-bridge, OpenVINS build layers
Makefile                                       + LOCALIZATION ?= gps, threaded into mission/flight-test
```

`common_control`/`common_missions`: **unmodified**.

---

## Milestone B implementation findings (real OpenVINS)

Same research-before-code discipline as Milestone A — several assumptions
that looked reasonable on paper turned out wrong when actually checked
against the live SITL instance and real package metadata. Recorded here so
Phase 4 hardware work doesn't have to re-derive them.

1. **Humble's DEFAULT `ros-humble-ros-gz-bridge` (from packages.ros.org) is
   the WRONG package** — it links against `libignition-transport11`
   (Gazebo **Fortress**), confirmed via `apt-cache depends`. px4-sim's
   Gazebo Harmonic 8.14.0 speaks `gz-transport13`, a different, incompatible
   wire protocol. Installing the default package would have built and run
   with no error at all — `ros_gz_bridge` would start, the topic would
   exist — while silently receiving **zero messages**, since Fortress and
   Harmonic clients can't talk to each other. The fix: OSRF's own apt repo
   (`packages.osrfoundation.org/gazebo/ubuntu-stable` — the exact same
   repo/key `docker/Dockerfile.px4_sim` already uses for px4-sim) also
   publishes Harmonic-specific packages named `ros-humble-ros-gzharmonic-*`.
   Verified `ros-humble-ros-gzharmonic-bridge` depends on
   `libgz-transport13` — the correct match — before adding it anywhere.
2. **The OakD-Lite model's `camera_imu` sensor topic is DEAD — zero
   publishers.** PX4's own `GZGimbal.cpp` subscribes to
   `.../camera_link/sensor/camera_imu/imu` unconditionally (for gimbal
   stabilization), which makes the topic name show up in `gz topic -l` —
   but the actual `OakD-Lite/model.sdf` never declares an IMU sensor by that
   name (confirmed by reading the full SDF). `gz topic -i` on that topic
   showed "No publishers" — a dangling subscription only. Worked around at
   the time by using the flight controller's own `imu_sensor` at `base_link`
   instead (a legitimate real-world VIO pattern in general, but a sim/real
   *topic* mismatch here specifically: real hardware's `/camera/camera/imu`
   is the CAMERA's own IMU, not the flight controller's). Superseded by
   finding 6 below — the `d435i` swap gives the camera a real, working IMU
   of its own, closing this gap properly instead of routing around it.
3. **Camera intrinsics were DERIVED from the actual SDF kinematic chain, not
   copied from OpenVINS's `rs_d455` reference** (which is calibrated for
   different, real D455 hardware — directly reusing those numbers would
   have been actively wrong for this sim rig's different FOV/resolution).
   From `OakD-Lite/model.sdf`'s `IMX214` sensor (`horizontal_fov=1.204`,
   `1920x1080`): `fx=fy=1397.22` (Gazebo's simple pinhole camera assumes
   square pixels), `cx,cy=960,540`, no distortion. **The camera-IMU
   extrinsic derived alongside it was WRONG, and this was a real,
   reproduced bug, not just a simplification** — see finding 6.
4. **OpenVINS's ROS 2 dependencies were resolved via `rosdep`, not
   hand-listed.** Verified live (cloned OpenVINS, ran
   `rosdep install --from-paths src --ignore-src -y --simulate`) before
   writing the Dockerfile layer — this is the standard, maintainable way to
   build a third-party ROS 2 package from source (reads *their*
   `package.xml`, not our guess of it). Resolved:
   `ros-humble-cv-bridge`, `libopencv-dev`, `libopencv-contrib-dev`,
   `libboost-all-dev`, `libceres-dev`, `ros-humble-image-transport`.
5. **A `RUN apt-get install ... && rm -rf /var/lib/apt/lists/*` layer wipes
   the apt package index for every SUBSEQUENT layer**, not just that one —
   Docker layers don't share filesystem state going forward the way you'd
   intuitively expect from "it's still the same image." The OpenVINS
   `rosdep install` layer failed with `Unable to locate package
   ros-humble-cv-bridge` because the *previous* layer's cleanup had already
   removed the apt cache, and `rosdep update` (which only refreshes
   rosdep's own dependency-key database) does not imply `apt-get update`
   (which refreshes apt's package index — a completely different cache).
   Fixed by adding an explicit `apt-get update` at the start of the
   `rosdep install` layer.
6. **A stationary hover flight-tested with real OpenVINS diverged and had
   to be force-disarmed** — `vehicle_local_position` drifted to
   `x=-7.3, y=-24.9` m and climbing, with `xy_reset_counter: 7`
   (`px4-commander land`/`AUTO_LAND` did not recover it). Root cause: the
   `T_cam_imu` rotation in `kalibr_imucam_chain.yaml` was identity —
   inherited from finding 3's "the whole chain is pure translation, no
   rotation anywhere" reasoning, which is TRUE of the SDF's declared `<pose>`
   chain but FALSE of the actual geometric relationship OpenVINS needs.
   A camera's SDF `<pose>` is declared in the link's own FLU-like body
   convention, but the pixel-projection ("optical") frame every pinhole VIO
   system (OpenVINS included) assumes is always +Z-forward/+X-right/+Y-down
   — a fixed ~90°/90° rotation away from body FLU for any forward-facing
   camera, independent of translation and independent of whatever `<rpy>`
   (or lack of it) appears in the SDF chain. "No `<rpy>` anywhere in the
   SDF" means the *SDF-declared pose* has zero rotation, not that the
   *body→optical* relationship does — those are two different rotations,
   and the design and initial implementation conflated them. Confirmed by
   deriving `R_cam_imu = [[0,-1,0],[0,0,-1],[1,0,0]]` (body +X→cam +Z,
   body +Y→cam −X, body +Z→cam −Y) and fixing it in
   `kalibr_imucam_chain.yaml`.

   Fixed alongside a second change made at the same time, which also
   simplified the extrinsic: **swapped the simulated camera from the stock
   `OakD-Lite` to our own local `d435i` model**
   (`docker/px4_sitl_models/d435i/model.sdf`, overlaid into the image by
   `Dockerfile.px4_sim` the same way `vio_test.sdf` is — the upstream
   PX4-Autopilot checkout is never edited directly, only overwritten
   post-clone at build time) so sim flies the actual camera Phase 4
   hardware uses, not a different one wearing matched topic names. The new
   model also gives the camera a real, working onboard IMU (co-located with
   the color sensor, fixing finding 2's dangling-`camera_imu` workaround
   properly instead of routing around it via the flight controller's IMU),
   which incidentally zeroes out `T_cam_imu`'s translation component too —
   the only remaining nontrivial part of the extrinsic is the rotation
   above. See `config/openvins/kalibr_imucam_chain.yaml`'s own header for
   the full derivation, kept in sync with this file.
7. **Finding 6's rotation had the right axes but the wrong direction —
   caught by a real `square` mission flight, not by re-deriving on paper.**
   With `calib_cam_extrinsics: true`, OpenVINS treats the YAML's `T_cam_imu`
   as a seed it refines online; the flight log showed it converging to and
   holding the exact TRANSPOSE of finding 6's matrix for the entire flight.
   Waypoint navigation flew fine on the self-corrected value, but the
   landing phase's aggressive combined dynamics outran the online
   correction and the filter genuinely diverged
   (`x=11.4, y=-10.8, z=4.97` before a forced disarm). Fixed by seeding the
   empirically-converged value directly and freezing it
   (`calib_cam_extrinsics: false`) — removing the whole "online correction
   lags under aggressive dynamics" failure class, not just this instance.
   Retested clean through both waypoints and the previously-diverging
   descent. Full incident: `DEVELOPMENT_STATUS.md`, "Milestone B result,
   part 3."
8. **Post-landing position/velocity estimate drift blocked auto-disarm —
   fixed at the `common_control` level, not the VIO level.** Two
   motion-threshold-based fixes inside OpenVINS were tried and reverted
   (re-enabling ZUPT mid-flight self-locks the estimate at the exact start
   of every takeoff, since v=0 there by definition — a structural gap in
   OpenVINS's ZUPT flags, not a tuning mistake). Fixed instead in
   `offboard_control_node.py`: subscribes to `vehicle_land_detected` and
   disarms (with PX4's required `param2=21196` force flag — a plain disarm
   is silently `MAV_RESULT_TEMPORARILY_REJECTED` otherwise, confirmed via
   `vehicle_command_ack`) once `has_low_throttle` — a pure actuator-thrust
   signal, confirmed via `MulticopterLandDetector.cpp` to be independent of
   the drifting estimate — holds sustained for a required dwell time, with
   a max-timeout ceiling that falls back to manual intervention rather than
   ever guessing from elapsed time alone. `DEVELOPMENT_STATUS.md` part 6.
9. **That same fallback-disarm testing surfaced a MORE SEVERE problem, then
   root-caused and fixed**: PX4's own `AUTO_LAND` controller, not this
   project's code, flew the vehicle away/upward (confirmed via Gazebo
   ground truth) when OpenVINS's estimate diverged badly during descent.
   Three real, independently-verified contributing causes: (1) the 0.2 m/s
   mission cruise speed starved OpenVINS of IMU excitation for its whole
   flight — confirmed via its own debug log that accelerometer bias never
   converged before landing in a diverged run, vs. converged in an earlier
   clean run at 1.0 m/s; raised missions to 0.8 m/s. (2) `EKF2_EV_POS_X/Y/Z`
   (the vision sensor's lever arm from the flight controller's own IMU) was
   never configured anywhere in this repo, despite the `d435i` model's
   camera-IMU being physically offset from `base_link` by the `CameraJoint`
   mount pose — now set automatically in `set_localization_source.py` on
   vision-source switch. (3) `vio_test.sdf`'s ground plane was flat gray,
   starving the tracker of features at low altitude (the tall fence props'
   tops exit the D435i's vertical FOV near the ground) — added a 64-tile
   ground-level checkerboard. Retested clean twice, fully automatic,
   ground-truth-verified. `DEVELOPMENT_STATUS.md`, "Milestone B result,
   part 7," including an honest caveat on remaining run-to-run variance.

---

## Non-Goals (explicitly deferred)

- **Live/runtime mid-flight source switching.** This phase is a launch-time
  choice only. A future live-failover mode (e.g. auto-switch to VIO on
  GPS-quality degradation) is a separate, later design.
- **Real hardware execution.** `hw_bringup`'s VIO wiring ships updated but
  still untested — consistent with its existing Phase-4 stub status.
- **Exact sim/real depth-image parity** (units/encoding) — not needed since
  VIO doesn't consume depth; noted, not solved.
- **A typed ROS 2 service / custom interfaces package.** Deliberate
  simplicity choice — this would be this repo's first `ament_cmake` package
  (real new build-system infra) for a feature explicitly meant to stay
  simple. A clean future refinement if live-switching is designed later.
- **SLAM/Nav2** (Phase 5, unchanged).

## Verification plan

**Milestone A (loopback — proves the mechanism, no camera/OpenVINS needed):**
1. `make mission MISSION=square LOCALIZATION=gps` — confirm unchanged
   behavior (regression check; `GPS_CTRL=7`/`EV_CTRL=0` is a no-op vs today).
2. `make mission MISSION=square LOCALIZATION=vision` with the loopback
   bridge active — confirm via `estimator_status_flags` that `cs_ev_pos`/
   `cs_ev_vel` go true and `cs_gnss_pos`/`cs_gnss_vel` go false, and the
   mission flies its full waypoint sequence and lands normally.
3. Repeat for `MISSION=survey LOCALIZATION=vision`.

**Milestone B (real OpenVINS):**
4. Bring up `ros_gz_bridge`, confirm `/camera/camera/color/image_raw` and
   `/camera/camera/imu` flow at healthy rates.
5. Launch OpenVINS standalone (vehicle not flying), confirm `ov_msckf/odomimu`
   publishes and looks sane while stationary.
6. Swap the loopback bridge for `openvins_odometry_bridge.py`, fly
   `make mission MISSION=square LOCALIZATION=vision` for real. Expect this
   step to need real tuning (EKF2 vision noise params, OpenVINS init
   settling) — iteration, not a guaranteed one-shot.
