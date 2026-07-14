# Mission Testing Guide

> Part of the [GPS-Denied Autonomous Drone Stack](../README.md) documentation set.

How to fly, once the stack is up and running — see the
[Setup Guide](setup-guide.md) first if it isn't yet. Covers waiting for PX4
to be ready, the hover test, flying a full mission, inspecting the running
system, and shutting down cleanly.

## Contents

- [4.5 Wait for PX4 to Be Ready to Arm](#sec-4-5)
- [4.6 Fly the Hover Test](#sec-4-6)
- [4.7 Fly a Mission](#sec-4-7)
- [4.8 Inspect the Running System](#sec-4-8)
- [4.9 Stop the Stack](#sec-4-9)

---

<a id="sec-4-5"></a>

### 4.5 Wait for PX4 to Be Ready to Arm

PX4's EKF2 state estimator needs **~30–60 seconds** after boot to converge
before it will allow arming — this is normal PX4 behavior, not something
wrong with this repo ([issue 8](known-issues.md#issue-8)). You don't have to do anything: every
flight command below retries once per second until PX4 accepts. If you want
to watch convergence yourself:
```bash
make shell
ros2 topic echo /fmu/out/vehicle_status_v1 --once
# wait for: pre_flight_checks_pass: true
```

<a id="sec-4-6"></a>

### 4.6 Fly the Hover Test

In a second terminal (leave the sim running in the first):
```bash
make flight-test
```

Runs `ros2 launch sim_bringup sim.launch.py action:=hover`, which:

1. Streams the `OffboardControlMode` heartbeat + `TrajectorySetpoint` at 10 Hz
2. Switches PX4 to offboard mode and arms (retrying once per second until
   PX4 confirms via `vehicle_status` — see [§4.5](mission-testing.md#sec-4-5))
3. Takes off to 2 m, hovers 5 s, lands, and disarms

Expected output ends with:
```
[INFO] [...]: Armed and offboard — climbing to takeoff height
[INFO] [...]: Reached takeoff height — hovering for 5.0s
[INFO] [...]: Hover complete — landing
[INFO] [...]: Landed and disarmed — mission complete
```
(For `LOCALIZATION=vision VIO_BACKEND=openvins`, you may instead see a
`WARN` line about the "low-throttle fallback" engaging just before that —
that's expected and by design, not an error; see [§8](reference.md#sec-8).)

By default this flies GPS. To fly it GPS-denied instead (needs
`PX4_GZ_WORLD=vio_test`, [§4.3](setup-guide.md#sec-4-3)):
```bash
LOCALIZATION=vision VIO_BACKEND=openvins make flight-test
```
See [§8](reference.md#sec-8) for what these two variables do.

<a id="sec-4-7"></a>

### 4.7 Fly a Mission

```bash
make mission                    # square, the default and currently only mission, GPS
LOCALIZATION=vision VIO_BACKEND=openvins make mission MISSION=square    # GPS-denied
```

Runs `ros2 launch sim_bringup sim.launch.py action:=mission
mission:=<name>`. `square` (exact geometry — `sim_params.yaml`, [§7](reference.md#sec-7)) is the
only mission in the repo right now — a square flight path **centered on
the takeoff point** (corners at ±`side_length_m`/2, so ±1.5 m with the
default 3 m side), nose pointed along each leg, with an explicit final
waypoint back to the center before landing. Centered, not first-quadrant,
since 2026-07-13: the original route flew out of one corner of the
`vio_test` fence area and landed only 1.5 m from two fence-prop lines —
with mono-VIO's known stochastic drift, observed flights hit the props
during landing. The world's fence/tiles were recentered on the origin at
the same time, so the landing point now has 4.75 m of clearance in every
direction and the flight corners keep ≥3.25 m. (A `survey`
lawnmower-coverage mission existed earlier but was removed 2026-07-09 —
its indoor test geometry didn't suit its own footprint; a
differently-shaped mission against a purpose-built world is planned
separately, not a retraction of the pattern itself.)

`square` is a subclass of `MissionBase`, which reuses the entire hover-test
arm/offboard/takeoff/land state machine and only supplies waypoints — a new
mission is ~25 lines (declare geometry parameters, return a waypoint list;
see `ros2_ws/src/common_missions/`). Waypoint arrival is judged by position
tolerance (0.5 m default), never by elapsed time, so missions behave
identically on slow and fast hosts. Mission geometry and control tuning live
in `sim_bringup/config/sim_params.yaml` ([§7](reference.md#sec-7)) — **every one of these values is
required, with no hardcoded fallback in the Python**: edit the YAML and
re-run `make mission`, no rebuild needed ([§4.4](setup-guide.md#sec-4-4)). If a value is missing from
the YAML, the node fails loudly at startup with a `[FATAL]` log naming the
exact missing parameter, instead of silently flying an unintended default.

<a id="sec-4-8"></a>

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
See [§6](reference.md#sec-6) for the full topic list this stack
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

<a id="sec-4-9"></a>

### 4.9 Stop the Stack

```bash
make stop
```

**Always restart the whole stack together** (`make stop && make sim`, or
`make stop && make sim-gui`). Restarting only the `px4-sim` container leaves
the DDS bridge wedged — see [issue 5](known-issues.md#issue-5).

---

**See also:** the full [Command](reference.md#sec-5),
[Topics](reference.md#sec-6), and [Parameters](reference.md#sec-7) reference
for tuning a mission or writing a new one.

[← Back to README](../README.md)
