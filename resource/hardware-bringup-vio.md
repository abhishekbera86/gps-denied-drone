# Hardware Bring-Up, Phase 2: VIO

> Part of the [GPS-Denied Autonomous Drone Stack](../README.md) documentation set.

**Status: UNTESTED**, same caveat as
[Phase 1](hardware-bringup-gps.md#what-you-need) — nothing here has flown.
This phase additionally depends on real camera-IMU calibration output that
doesn't exist yet (the `kalibr_imucam_chain_hw.yaml`/
`kalibr_imu_chain_hw.yaml` files this repo ships are explicit templates,
not real values — see [step 5](#calibration)).

**Prerequisite: finish [Phase 1: GPS](hardware-bringup-gps.md) first** and
have it flying reliably. This phase adds a real, currently-open failure
mode (VIO accuracy — see [Known Issues #17-28](known-issues.md#issue-17)
for everything that went wrong tuning this in sim, all still relevant
here) on top of hardware that itself has never flown. Isolate that risk
the same deliberate way Phase 1 isolated `hw_bringup` itself from VIO's
open questions.

## Contents

1. [Mount the camera and measure the lever arm](#mount)
2. [Verify the D435i on the Orange Pi](#verify-camera)
3. [Camera-IMU calibration with Kalibr](#calibration)
4. [Fill in the real calibration files](#fill-in-config)
5. [Bench-test the VIO pipeline](#bench-vio)
6. [Prepare an indoor test area](#test-area)
7. [First restrained VIO hover](#restrained-vio)
8. [First free VIO hover, then a mission](#free-vio)
9. [Deciding when to freeze the extrinsics](#freeze-extrinsics)
10. [Troubleshooting](#troubleshooting)

---

<a id="mount"></a>

## 1. Mount the camera and measure the lever arm

Mount the D435i on the frame facing forward, as rigidly as your frame
allows — any flex between the camera and the flight controller shows up
as VIO noise no calibration can fix. Angle: this repo's sim history has
one open, unresolved data point worth knowing before you commit to a tilt
— a 30° downward-tilted mount was tried on a separate branch, flew once,
and diverged 84m (never root-caused, that branch was never merged).
**Mount level (forward-facing, no tilt) for this first real-hardware
attempt** — don't repeat an experiment this project's own history flags
as open and risky before the untilted baseline itself is even proven on
real hardware.

**Measure the lever arm** — the D435i's own onboard IMU position relative
to the Pixhawk's IMU, in **FRD body frame, meters** (X forward, Y right, Z
down, from the flight controller's IMU to the camera's IMU). This is
airframe-specific physical geometry with no substitute for actually
measuring your particular frame:

1. Find the D435i's own IMU location on the camera — check Intel's D435i
   mechanical drawing (not the color sensor's location; they're offset by
   a few mm on real hardware, unlike sim's model which deliberately
   co-located them).
2. Measure from the Pixhawk 6C's IMU location (check its own mechanical
   drawing for the exact chip location, not just "the board") to that
   point, along the vehicle's forward/right/down axes.
3. You'll use these three numbers in [step 8's restrained
   test](#restrained-vio) and beyond, passed as
   `EV_POS_X`/`EV_POS_Y`/`EV_POS_Z` to `make hw-flight-test`/
   `make hw-mission` (see [Reference](reference.md) —
   `set_localization_source.py` was extended specifically for this
   guide with `--ev-pos-x/y/z` overrides; the sim values baked into that
   file's `VISION_LEVER_ARM_FRD` are for the simulated d435i model's
   co-located mount and do NOT apply to your physical frame).

Get this right before flying vision mode at all — a wrong lever arm was a
confirmed, real contributor to an in-flight divergence in this project's
own sim history ([Known Issues #18](known-issues.md#issue-18)), and that
was for a MUCH smaller, precisely-known simulated offset than a
hand-measured real mount is likely to have.

<a id="verify-camera"></a>

## 2. Verify the D435i on the Orange Pi

**Host-side udev rules** (on the Orange Pi itself, outside any
container) — librealsense's documented requirement for non-root USB
access, and the counterpart to the same rules file `Dockerfile.hw_autonomy`
installs inside the container (both are needed together, neither alone is
sufficient — a container-only udev rule doesn't affect how the host
kernel presents the device node in the first place):

```bash
# on the Orange Pi host, not inside any container
git clone --depth 1 https://github.com/IntelRealSense/librealsense.git /tmp/librealsense
sudo cp /tmp/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
rm -rf /tmp/librealsense
```

Plug in the D435i, then confirm the host sees it before trusting anything
inside Docker:

```bash
lsusb | grep -i intel
```

Then verify inside the container:

```bash
make shell-hw
source /opt/ros/humble/setup.bash
source /opt/realsense_ws/install/setup.bash
ros2 run realsense2_camera realsense2_camera_node --ros-args \
    -p enable_color:=true -p enable_gyro:=true -p enable_accel:=true \
    -p unite_imu_method:=2 &
sleep 3
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/imu
ros2 topic echo /camera/camera/color/camera_info --once
```

If either topic never produces data, stop here — this is a USB/udev
problem, not a VIO problem, and needs fixing before anything downstream
can work. `docker-compose.yml`'s `hw-autonomy` service runs `privileged:
true` with `/dev/bus/usb` mounted specifically for this (see that file's
own comment for why `privileged` was chosen over a narrower device list);
if the camera still doesn't enumerate inside the container despite the
host seeing it, that mount/privilege setup is the first thing to
re-check.

**`camera_info`'s intrinsics are real hardware fact, not something to
derive** — unlike sim (where intrinsics had to be calculated from
Gazebo's synthetic pinhole model,
[Known Issues #3 in the localization design doc](phase3-gps-denied-localization-source.md)),
the physical D435i ships factory-calibrated and `realsense-ros` publishes
its real intrinsics on `/camera/camera/color/camera_info` automatically.
Note the `fx`, `fy`, `cx`, `cy`, and distortion values from the `echo`
above — you'll need them in [step 4](#fill-in-config).

<a id="calibration"></a>

## 3. Camera-IMU calibration with Kalibr

This is the step sim never needed (its extrinsics were derivable from
known SDF geometry — see
[Known Issues #22](known-issues.md#issue-22)/[#23](known-issues.md#issue-23)
for how much even a *derivable* extrinsic still went wrong in practice).
Real hardware has no such shortcut: the camera-to-its-own-IMU rotation
and the camera-IMU hardware time offset (`timeshift_cam_imu`) both need
actual measurement.

[Kalibr](https://github.com/ethz-asl/kalibr) is the standard tool for
this (it's what produced the `kalibr_imucam_chain.yaml`/
`kalibr_imu_chain.yaml` file FORMAT this project already uses — the sim
config files are written in Kalibr's own output format even though sim's
particular values were derived analytically rather than run through
Kalibr itself). Kalibr is not part of this repo (a calibration tool run
once, not a flight-time dependency) — run it in its own official Docker
container on any machine, not necessarily the Orange Pi:

```bash
docker pull kalibr/kalibr:latest    # or build from ethz-asl/kalibr's own Dockerfile
```

1. **Print a calibration target** — Kalibr's own April-tag grid
   (`aprilgrid`) is the standard choice; generate one from Kalibr's own
   tools and print it flat and rigid (mount the printout on a stiff board
   — a target that flexes ruins the calibration).
2. **Record a calibration bag** — with the D435i's `/camera/camera/
   color/image_raw` and `/camera/camera/imu` both publishing (same
   commands as [step 2](#verify-camera)), record a `ros2 bag` while
   slowly moving the camera+IMU rig by hand in front of the target,
   covering a wide range of angles and a few different distances, with
   plenty of rotational AND translational motion (Kalibr's own
   documentation covers the exact motion pattern needed — this is not
   optional or approximate; insufficient excitation is the most common
   reason a Kalibr calibration silently converges to a bad answer).
3. **Convert the ROS 2 bag to what Kalibr expects** (Kalibr's tooling is
   historically ROS 1-centric) — check Kalibr's own current
   documentation for the ROS 2 bag workflow, since this detail changes
   across Kalibr versions and is exactly the kind of thing to verify
   live rather than assume.
4. **Run the camera-IMU calibration**:
   ```bash
   kalibr_calibrate_imu_camera \
       --bag your_calibration.bag \
       --cam camchain.yaml \
       --imu imu.yaml \
       --target aprilgrid.yaml
   ```
   (Run `kalibr_calibrate_cameras` first if you don't already have a
   `camchain.yaml` intrinsics/distortion result — or use the real
   `camera_info` values from [step 2](#verify-camera) directly, since
   those are already the real factory calibration.)
5. Kalibr outputs a `camchain-imucam.yaml` (or similar) containing the
   real `T_cam_imu` and `timeshift_cam_imu` — this is the actual output
   you transcribe into `kalibr_imucam_chain_hw.yaml` in the next step.

<a id="fill-in-config"></a>

## 4. Fill in the real calibration files

Three files, all under
`ros2_ws/src/common_perception/config/openvins/`, all currently
templates with placeholder/starting-point values and explicit `FILL IN`
markers in their own comments:

- **`kalibr_imucam_chain_hw.yaml`**: `T_cam_imu` (rotation AND
  translation — do not assume translation is zero the way sim could,
  see that file's own header) and `timeshift_cam_imu` from
  [step 3](#calibration)'s Kalibr output; `intrinsics`/`resolution`/
  `distortion_coeffs` from [step 2](#verify-camera)'s real
  `camera_info` — at the SAME resolution you calibrated with and intend
  to fly with (changing resolution after calibrating invalidates these
  numbers, the exact mistake class documented in this project's own sim
  history for a different reason — see that file's header comment).
- **`kalibr_imu_chain_hw.yaml`**: the noise-density values already in
  this file are published D435i/BMI055 figures (cited in the file's own
  header), a reasonable starting point — not a substitute for your own
  Allan-variance characterization once the airframe is otherwise flying
  reliably (that file's header names the tools — `allan_variance_ros`
  or `imu_utils` — and the ~2-3 hour static recording needed). Confirm
  `update_rate` matches what you actually configured
  (`ros2 topic hz /camera/camera/imu` from [step 2](#verify-camera)).
- **`hw_vio.launch.py`**: confirm `rgb_camera.color_profile` matches the
  resolution/rate you calibrated at, and double-check
  `unite_imu_method`'s exact accepted value against your installed
  realsense-ros version (`ros2 launch realsense2_camera rs_launch.py
  --show-args`) — that file's own header flags this as unverified
  against a live install.

<a id="bench-vio"></a>

## 5. Bench-test the VIO pipeline

Before touching the flight controller at all, confirm OpenVINS actually
initializes against your real calibration:

```bash
make shell-hw
source /opt/ros/humble/setup.bash
source /opt/px4_ros2_ws/install/setup.bash
source /opt/realsense_ws/install/setup.bash
source /opt/openvins_ws/install/setup.bash
source /ros2_ws_build/install/setup.bash
ros2 launch common_perception hw_vio.launch.py
```

Hold the rig still (matching OpenVINS's static-initializer requirement,
`try_zupt: true`/`init_window_time: 2.0` in `estimator_config_hw.yaml` —
same mechanism as sim, see [Reference](reference.md)) in a room with
real visual texture — a blank wall or ceiling gives the feature tracker
nothing to lock onto, the exact real-world version of the lesson sim's
own flat `empty` world already taught
([Known Issues #35/#36](known-issues.md#issue-35)). Watch for
`vio_output_check`'s pass/fail within 15s — same fail-loud guard as sim,
watching OpenVINS's actual output rather than raw camera/IMU presence,
for the same reason documented in that launch file. If it fails, work
through its printed error message's checklist before going further; do
not attempt a flight until this passes cleanly on the bench.

<a id="test-area"></a>

## 6. Prepare an indoor test area

- **Visual texture, not a blank room** — the same lesson as
  `vio_test.sdf`'s checkerboard fix
  ([Known Issues #23](known-issues.md#issue-23)): boxes, patterned floor
  tiles, furniture, anything with real corners/edges the KLT tracker can
  lock onto. A stark white-walled empty room is the real-world equivalent
  of sim's original `empty` world and will starve OpenVINS the same way.
- **Derive a real geofence limit for THIS space**, same procedure as
  [Phase 1's geofence step](hardware-bringup-gps.md#geofence) but for
  your indoor area specifically — indoor spaces are typically much
  smaller than an outdoor GPS test area, so re-measure, don't reuse the
  outdoor value.
- **Update `hw_params.yaml`'s `square_mission` section too, not just the
  hover profile** — this project has already hit the exact mistake of a
  geofence value added to one mission section and not the other once in
  sim ([Known Issues #34](known-issues.md#issue-34)'s "real bug found
  while wiring the param" note) — check both sections explicitly.

<a id="restrained-vio"></a>

## 7. First restrained VIO hover

Same restrained/tethered setup as
[Phase 1's restrained hover](hardware-bringup-gps.md#restrained-hover),
props on, vehicle physically unable to leave the ground:

```bash
make hw-flight-test LOCALIZATION=vision \
    EV_POS_X=<your measured value> EV_POS_Y=<your measured value> EV_POS_Z=<your measured value>
```

Watch `/fmu/out/estimator_status_flags` in a second shell
(`make shell-hw` again) to confirm vision is genuinely fusing, the same
verification sim's own [Reference §8](reference.md#sec-8) documents:

```bash
ros2 topic echo /fmu/out/estimator_status_flags --once
# cs_gnss_pos: false   cs_gnss_vel: false   (GPS genuinely off)
# cs_ev_pos: true      cs_ev_vel: true      (vision genuinely fusing)
```

If the mission gets stuck waiting for arm/offboard the way this project's
own sim side once did for an unrelated reason
([Known Issues #35](known-issues.md#issue-35)), don't assume it's the
same root cause — that one was a sim-only world/topic mismatch that
cannot occur on real hardware (there's no Gazebo world to mismatch
against here). Work through `vio_output_check`'s error message and
[this guide's troubleshooting](#troubleshooting) instead.

<a id="free-vio"></a>

## 8. First free VIO hover, then a mission

Once the restrained test is clean and `estimator_status_flags` confirms
genuine vision fusion, repeat [Phase 1's free-hover](hardware-bringup-gps.md#first-hover)
and [first-mission](hardware-bringup-gps.md#first-mission) procedure with
`LOCALIZATION=vision` and your real `EV_POS_*` values. Track the
vehicle's actual position independently (marked positions on the floor, a
tape measure, or a second phone/camera) and compare against what the
mission log reports — this is the real-hardware equivalent of sim's own
"confirmed via Gazebo ground truth" verifications throughout
[Known Issues #21-25](known-issues.md#issue-21), which you don't get for
free on real hardware the way sim's own ground-truth pose did.

**Known, still-open limitation carried over from sim, applies here too**:
post-landing position/velocity ESTIMATE drift
([Known Issues #21](known-issues.md#issue-21)) — the vehicle should land
and disarm cleanly via the low-throttle fallback either way (source-
agnostic, GPS or vision, same code), but don't be alarmed if you see a
`WARN` about the fallback engaging rather than PX4's own auto-disarm
firing — that's expected and by design, not a new bug specific to your
hardware.

<a id="freeze-extrinsics"></a>

## 9. Deciding when to freeze the extrinsics

`estimator_config_hw.yaml` ships with `calib_cam_extrinsics: true`
(online refinement), deliberately NOT sim's frozen `false` — see that
file's own comment for why: sim could only freeze its value after
independently re-deriving it AND confirming it against a real flight log
across multiple flights ([Known Issues #22](known-issues.md#issue-22)).
Your Kalibr output has no such independent cross-check yet.

Only set `calib_cam_extrinsics: false` once flight logs across **several**
flights (not one) show OpenVINS's live-refined `T_cam_imu` converging to
and holding the **same** value every time — grep OpenVINS's own debug
output for `cam0 extrinsics` across multiple flight logs, the same
evidence sim's own freeze decision was based on. A value that moves
around between flights, or converges differently under aggressive
maneuvers (sim's own AUTO_LAND divergence, found the same way — see
[Known Issues #22](known-issues.md#issue-22)) means your Kalibr seed or
your physical mount isn't rigid/accurate enough yet — fix that before
freezing anything.

<a id="troubleshooting"></a>

## 10. Troubleshooting

- **`vio_output_check` fails, no camera data at all**: re-check
  [step 2](#verify-camera) — this is a USB/udev problem before it's ever
  a calibration problem.
- **Camera data flows but OpenVINS never initializes**: check visual
  texture ([step 6](#test-area)) and confirm you're actually holding the
  rig still for the full `init_window_time` — same static-initializer
  requirement as sim
  ([Known Issues #29](known-issues.md#issue-29)'s init-window discussion,
  though that entry's specific SITL-clock cause cannot recur on real
  hardware).
- **Estimate diverges under aggressive movement but flies fine
  smooth/slow**: this is the exact shape of
  [Known Issues #22](known-issues.md#issue-22)'s extrinsics-direction bug
  — don't freeze `calib_cam_extrinsics` yet if you see this ([step
  9](#freeze-extrinsics)), and re-verify your Kalibr calibration's
  quality (recalibrate with more/better-varied motion in the recording).
- **Something not covered here**: same note as
  [Phase 1's troubleshooting section](hardware-bringup-gps.md#troubleshooting)
  — this phase is even less verified than that one. Document what you
  find in [Known Issues & Fixes](known-issues.md).

---

[← Back to README](../README.md) · [Phase 1: GPS](hardware-bringup-gps.md)
