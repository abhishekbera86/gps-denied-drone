# GPS-Denied Autonomous Drone — Implementation Plan (v3 — PX4-native ROS 2)

> **Strategy:** PX4 + Gazebo Harmonic + ROS 2 Humble, talking over PX4's own
> uXRCE-DDS bridge (`px4_msgs`/`px4_ros_com`) — no MAVSDK, no Aerostack2.
> Fully Dockerized: nothing installed on the dev host or on the Orange Pi 5
> Plus / Pixhawk 6 companion computer.

This supersedes the earlier Aerostack2-based plan (v2). That approach hit real
friction — mission code broke against undocumented, idempotency-guarded AS2
service semantics, and the Makefile/compose drifted from the plan doc after a
mid-project pivot away from PX4/Gazebo. v3 is a clean redesign: every
behavior is code we write and can read, no third-party hidden state machine.

---

## Objective

A **GPS-denied autonomous quadcopter** that:
- Flies using Visual-Inertial Odometry (D435i + OpenVINS), added in Phase 3
- Runs on **PX4 v1.17.0 + ROS 2 Humble**, controlled over PX4's native
  **uXRCE-DDS** bridge (no MAVLink translation hop)
- Is **fully containerized** (Docker Compose only — zero host install, zero
  install on the Orange Pi 5 Plus companion computer)
- Uses **the same autonomy code** in simulation (Gazebo Harmonic) and on the
  real drone (Orange Pi 5 Plus + Pixhawk 6)
- Supports **multiple pluggable missions**, not one hardcoded flight path
- Leaves a clean extension point for **SLAM + Nav2** once basic flight and
  VIO are solid

---

## Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                     common_autonomy (ROS 2 workspace)                │
│           SHARED — identical source, sim and real hardware          │
│                                                                      │
│  common_control/      OffboardControlNode: arm/takeoff/setpoint/    │
│                        land state machine over px4_msgs topics      │
│  common_missions/      Pluggable mission classes (square, survey…)  │
│  common_perception/    VIO launch + topic remapping (sensor-agnostic)│
└───────────────────────────────┬──────────────────────────────────────┘
                                 │  identical ROS 2 topics
                                 │  (/fmu/in/*, /fmu/out/*, /vio/odometry…)
                 ┌───────────────┴────────────────┐
                 │                                │
        ┌────────▼─────────┐            ┌─────────▼──────────┐
        │   sim_bringup     │            │    hw_bringup       │
        │  (dev ThinkPad,   │            │  (Orange Pi 5+,     │
        │   x86_64, no GPU) │            │   ARM64)            │
        │                   │            │                     │
        │  px4-sim:         │            │  ros2-autonomy:     │
        │   PX4 v1.17.0     │            │   uxrce_dds_agent   │
        │   SITL + Gazebo   │            │   (serial→Pixhawk 6)│
        │   Harmonic        │            │                     │
        │   (x500_depth,    │            │  hw-sensors:        │
        │   headless)       │            │   realsense-ros     │
        │                   │            │   (D435i, USB3,     │
        │  ros2-autonomy:   │            │   privileged)        │
        │   (same image)    │            │                     │
        │                   │            │  vio: (same image)  │
        │  vio: (same image)│            │                     │
        └───────────────────┘            └─────────────────────┘
```

**Rule:** all flight/mission logic lives in `common_autonomy`; `sim_bringup`/
`hw_bringup` only contain launch files, world/SDF files, and connection
config (UDP vs serial). The only difference between sim and real is the
uXRCE-DDS transport endpoint and the sensor source topic — **never** the
mission code.

### Why PX4-native ROS 2 instead of Aerostack2 or MAVSDK

| | PX4-native ROS 2 (chosen) | MAVSDK | Aerostack2 |
|---|---|---|---|
| Transport | uXRCE-DDS, direct ROS 2 topics | MAVLink | AS2's own platform/behavior layers over MAVLink |
| Hidden state | None — every behavior is our own code | Some (SDK-internal) | Yes — idempotency-guarded services, multiple plugin layers (root cause of the v2 rewrite) |
| Nav2 integration later | Native — no bridge needed | Needs a MAVSDK↔ROS 2 bridge node | Needs AS2's own Nav2 integration |
| Official PX4 support | Yes, PX4's own recommended ROS 2 Humble path | Yes (separate SDK) | Third-party framework |

### ROS 2 Workspace Layout

```
ros2_ws/src/
  common_control/       OffboardControlNode (px4_msgs: TrajectorySetpoint,
                         VehicleCommand, OffboardControlMode, VehicleStatus)
                         — arm, takeoff, goto(x,y,z,yaw), land, disarm primitives
  common_missions/       MissionBase + concrete missions (square_mission.py,
                         survey_mission.py, hover_mission.py…), selected via a
                         `mission:=` launch argument, dict-registry lookup —
                         no framework-level mission API
  common_perception/      VIO launch/config + a topic-remap layer so
                         common_control never knows whether odometry came
                         from Gazebo-simulated camera or the real D435i
  sim_bringup/           Gazebo world, PX4 SITL launch args, ros_gz_bridge
                         config mapping simulated camera/IMU → standard topics
  hw_bringup/            realsense-ros launch + uXRCE-DDS agent serial launch
                         for the Orange Pi
```

### Mission plugin design
`common_missions.MissionBase` defines `run(control: OffboardControlNode) -> bool`.
Each mission is a small class registered in a plain dict by name
(`{"square": SquareMission, "survey": SurveyMission, ...}`), loaded via a
launch argument. Adding a new mission = adding one file.

### Sim-to-real sensor parity
The `x500_depth` Gazebo model publishes a simulated depth+IMU stream;
`ros_gz_bridge` republishes it under the same topic names/types
(`sensor_msgs/Image`, `CameraInfo`, `Imu`) that `realsense-ros` produces on
real hardware. `common_perception`'s VIO launch subscribes to those fixed
topic names regardless of which bringup is active.

---

## Version Pins

| Component | Version | Why |
|---|---|---|
| PX4-Autopilot | `v1.17.0` | Latest stable release; first stable line with native Gazebo Harmonic support (v1.16+). Flash this same version to the physical Pixhawk 6 in Phase 4. |
| `px4_msgs` | `release/1.17` | Exact match to PX4 firmware — message-definition mismatches cause silent DDS failures. |
| `px4_ros_com` | `release/1.16` | No `release/1.17` branch exists yet; this repo is mostly example/utility code, not tightly version-coupled to firmware. |
| Gazebo | Harmonic (via PX4's `Tools/setup/ubuntu.sh`) | PX4 v1.16+'s default new-Gazebo integration. |
| Micro-XRCE-DDS-Agent | `v3.0.1` | Bridges PX4's DDS client to ROS 2 topics. |
| ROS 2 | Humble | Required by the project; `ros:humble-ros-base` image. |
| librealsense | `2.58.2` | Must match physical D435i firmware (wired up in Phase 4). |

**Rejected:** PX4 v1.14.3 (the version referenced everywhere in the old v2
repo) — only supports Gazebo Garden, not Harmonic. Also rejected: the
prebuilt `px4io/px4-sitl-gazebo` Docker image — it ships PX4 v1.18.0-alpha1
with no path back to a stable, pinned version.

---

## Phase Checklist

### Phase 0 — PX4 SITL + Gazebo Harmonic smoke test ✅ COMPLETE
- [x] `docker/Dockerfile.px4_sim` — PX4 v1.17.0 SITL + Gazebo Harmonic, headless, `x500_depth` model
- [x] `docker/Dockerfile.ros2_autonomy` — ROS 2 Humble + Micro-XRCE-DDS-Agent + `px4_msgs`/`px4_ros_com`
- [x] `docker-compose.yml` — `px4-sim` + `ros2-autonomy` services, `sim` profile, host networking
- [x] `.env` — version pins, `x500_depth` model, RMW/DDS config
- [x] `Makefile` — `build`/`sim`/`stop`/`shell`/`shell-px4`/`logs`/`ps` (no dead targets)
- [x] px4-sim boots headless, Gazebo Harmonic 8.14.0 spawns `x500_depth_0`, no GPU
- [x] `uxrce_dds_client` connects to the agent at `127.0.0.1:8888`
- [x] `ros2 topic list` (inside `ros2-autonomy`) shows full `/fmu/in/*` and `/fmu/out/*` set
- [x] Live data confirmed: `/fmu/out/vehicle_odometry` streaming ~13 Hz

Two build-time bugs found and fixed here (see README "Known Issues" for detail):
1. `DONT_RUN=1` is not honored by the new Gazebo (`gz_x500_depth`) target —
   only the old Gazebo Classic/jMAVSim scripts check it. Fixed by
   pre-compiling via `make px4_sitl_default` instead (PX4's own CI build-only
   target).
2. `ros:humble-ros-base` doesn't ship `rmw_cyclonedds_cpp` — had to install
   `ros-humble-rmw-cyclonedds-cpp` explicitly.

### Phase 1 — Offboard hover (no VIO, no mission plugins yet) ✅ FLIGHT VERIFIED
- [x] `ros2_ws/src/common_control` package: `OffboardControlNode`
      (heartbeat + setpoint stream → offboard mode → arm → takeoff →
      hover → land), command-and-confirm with retry against
      `vehicle_status_v1` — never fire-and-forget
- [x] Runs against SITL using ground-truth EKF2 (GPS present in sim)
- [x] Full cycle verified end-to-end: arm → offboard → 2 m takeoff →
      4 s hover → AUTO_LAND → touchdown → auto-disarm
- [x] SITL-only PX4 param overrides required for arming, applied via
      `docker/px4_sitl_overrides/4002_gz_x500_depth.post` (never reaches
      real hardware): `CBRK_SUPPLY_CHK 894281` (no battery sim on
      x500_depth), `NAV_DLL_ACT 0` (no GCS attached; PX4 code default)
- [ ] Polish: `make flight-test` target (colcon build + run in one command)
- [ ] Polish: DDS-bridge wedge on px4-sim-only restart — full-stack restart
      is the workaround; consider lifecycle/healthcheck fix

### Phase 2 — Mission plugins
- [ ] `ros2_ws/src/common_missions` package: `MissionBase` + `square_mission.py`
      + `survey_mission.py`, selected via `mission:=` launch arg
- [ ] Still GPS-available in sim — proves the mission-plugin pattern works
      before GPS-denied complexity is added
- [ ] Verify: each mission runs by name, completes its waypoint sequence,
      disarms cleanly

### Phase 3 — GPS-denied VIO in sim
- [x] `ros2_ws/src/common_perception` package: localization-source switch
      (`set_localization_source`, MAVLink `PARAM_SET` — the uXRCE-DDS bridge
      cannot set PX4 params in this pinned version, see
      `resource/phase3-gps-denied-localization-source.md`) + OpenVINS launch
      + config
- [x] `common_perception`: `ros_gz_bridge` config mapping the `x500_depth`
      camera/IMU topics to the same names `realsense-ros` will use on
      hardware (lives in `common_perception`, not `sim_bringup`, since it's
      sensor-agnostic VIO plumbing, not sim-vs-hw connection config)
- [x] Feed VIO odometry into PX4 EKF2 (disable GPS fusion via
      `EKF2_GPS_CTRL=0`/`EKF2_EV_CTRL=5`) — chosen once at launch
      (`LOCALIZATION=gps|vision`), not a live in-flight switch (deferred)
- [x] Milestone A (loopback fake-VIO stand-in): rerun Phase 2 missions
      (square, survey) GPS-denied — both flew and landed normally
- [ ] Milestone B (real OpenVINS): rerun Phase 2 missions GPS-denied with
      real VIO — build wired up (`VIO_BACKEND=openvins`), full flight
      verification in progress/pending real-world tuning
- [x] Verify: `estimator_status_flags` confirms `cs_ev_pos`/`cs_ev_vel: true`,
      `cs_gnss_pos`/`cs_gnss_vel: false` when `LOCALIZATION=vision`

### Phase 4 — Real hardware (Orange Pi 5 Plus + Pixhawk 6C + D435i)
Infrastructure authored 2026-07-14 — everything below marked [x] exists in
the repo, but **nothing has run on real hardware yet**, so treat every
[x] as "written and reviewed," not "verified." Follow
resource/hardware-bringup-gps.md and resource/hardware-bringup-vio.md,
which supersede this checklist as the actual step-by-step path.
- [x] `docker/Dockerfile.hw_autonomy` — ARM64, librealsense v2.58.2 (from
      source, RSUSB backend — no ARM64 apt package exists), `realsense-ros`,
      plus the same DDS agent/px4_msgs/px4_ros_com/OpenVINS as sim's image,
      minus the Gazebo-only ros_gz_bridge layer. One combined image, not
      the originally-sketched separate `Dockerfile.hw_sensors` — see that
      Dockerfile's own header for why (no benefit to splitting DDS-agent
      and camera/VIO across two containers on one companion computer).
- [x] `hw_bringup`: uXRCE-DDS agent over serial to the Pixhawk 6C
      (`hw.launch.py`, pre-existing), RealSense + OpenVINS now actually
      wired in (`common_perception/launch/hw_vio.launch.py`, new) when
      `localization_source:=vision` — no `vio_backend` choice on real
      hardware, unlike sim (no fake-VIO loopback stand-in exists here).
- [x] `docker-compose.yml` `hw` profile (`hw-autonomy` service, privileged +
      USB/serial device passthrough) and `Makefile` hw targets
      (`build-hw`/`build-ws-hw`/`hw-flight-test`/`hw-mission`/`shell-hw`/
      `stop-hw`) — not in the original checklist below, added because the
      checklist items above need somewhere to actually run.
- [x] Real-hardware OpenVINS calibration file templates
      (`config/openvins/{estimator_config,kalibr_imucam_chain,
      kalibr_imu_chain}_hw.yaml`) — explicit placeholders, not real
      calibration output; filling them in via a real Kalibr calibration is
      resource/hardware-bringup-vio.md's central task.
- [x] `set_localization_source.py` gained `--ev-pos-x/y/z` CLI overrides —
      the sim d435i's lever arm was previously hardcoded module-level and
      had no way to express a different physical mount.
- [ ] Flash PX4 v1.17.0 to the physical Pixhawk 6C — resource/
      hardware-bringup-gps.md §2, not yet done (no hardware).
- [ ] Upload matching EKF2 GPS-denied params — mechanism already exists
      (`set_localization_source`, same code as sim) and needs no new work;
      "not yet done" here means not yet exercised against real firmware.
- [ ] Build ARM64 images on the Orange Pi 5 Plus itself (`make build-hw`) —
      Dockerfile exists (see above), never actually built on ARM64 hardware.
- [ ] Tethered hover test (props off first) — resource/
      hardware-bringup-gps.md §9-10.
- [ ] Run the identical Phase 2/3 mission code, unmodified, on real
      hardware — resource/hardware-bringup-gps.md §12 (GPS),
      resource/hardware-bringup-vio.md §8 (VIO).

### Phase 5 — SLAM + Nav2 (deferred, not designed in detail yet)
- [ ] Evaluate RTAB-Map as the mapping-capable successor/complement to
      OpenVINS (CPU-only, integrates with Nav2 directly) — decide once
      Phase 4 is solid, not before
- [ ] Nav2 output feeds `common_control` as plain ROS 2 topics — no bridge
      node needed, since everything upstream is already ROS 2-native

Each phase gates the next — don't start VIO wiring until plain offboard
hover is reliable, and don't start hardware until sim missions are reliable.
