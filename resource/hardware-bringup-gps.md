# Hardware Bring-Up, Phase 1: GPS

> Part of the [GPS-Denied Autonomous Drone Stack](../README.md) documentation set.

**Status: UNTESTED.** This guide, and the `hw-autonomy` Docker image /
`hw_bringup` launch files it walks you through, were authored 2026-07-14
against a repo that had never run on real hardware. Nothing here has been
flight-verified. Follow this guide the way every other part of this
project was actually brought up — assume nothing works until you've
watched it work, and expect to find and fix real bugs along the way (see
[Known Issues & Fixes](known-issues.md) for how many the *simulator* side
alone turned up). Treat every step as a checkpoint to verify, not a
formality to skip.

**Scope of this guide: GPS only, and NOT flying yet.** This guide takes
you from an unwired airframe to a fully verified, dry-run-tested system —
every sensor, every link, every failsafe checked with props OFF — before
[the last three steps](#restrained-hover) ever spin a motor in anger.
That ordering is deliberate: `hw_bringup` itself is a genuine unknown (no
ARM64 image has ever been built, no serial DDS link has ever been tried),
and the right way to burn down an unknown this size is bench verification
first, motors second, free flight third — never skip straight to
"let's just try it in the air." Once a GPS mission flies reliably and
repeatably, continue to [Phase 2: VIO](hardware-bringup-vio.md).

## Contents

1. [What you need](#what-you-need)
2. [Flash PX4 to the Pixhawk 6C](#flash-px4)
3. [Physical connections: Pixhawk ↔ Orange Pi](#wiring)
4. [Configure PX4 for GPS flight](#px4-config)
5. [Set up the Orange Pi](#opi-setup)
6. [Build the hardware image](#build)
7. [Derive a real geofence limit](#geofence)
8. [Dry run — verify everything before any prop spins](#dry-run)
9. [Restrained hover test — props on](#restrained-hover)
10. [First free hover](#first-hover)
11. [First GPS mission](#first-mission)
12. [Troubleshooting](#troubleshooting)

---

<a id="what-you-need"></a>

## 1. What you need

- The custom quad frame, assembled, with motors/ESCs already wired (this
  guide starts from "the airframe is mechanically complete," not frame
  assembly — that's airframe-specific and outside this repo's scope).
- Pixhawk 6C flight controller.
- Orange Pi 5 Plus, with a case/mount and a way to power it from the
  drone's battery (a 5V BEC/UBEC sized for the Orange Pi's actual draw —
  check its datasheet; an Orange Pi 5 Plus under ROS 2 + Docker load
  pulls meaningfully more than an idle board — budget for at least 3A at
  5V, more if you'll run the camera in Phase 2).
- An external GPS + compass module compatible with PX4 (whatever your
  Pixhawk 6C's GPS port expects — check its pinout).
- **Two USB cables** (see [step 3](#wiring) for why you need two, not
  one) — or one USB cable plus a USB-to-TTY serial adapter, depending on
  which wiring option you pick.
- A microSD card (or NVMe, if your Orange Pi 5 Plus carrier board
  supports it) for **Ubuntu 22.04** on the Orange Pi 5 Plus.
- Props, a battery, a safe open outdoor test area, and normal RC safety
  gear — an RC transmitter with a hardware kill switch is strongly
  recommended even though this stack flies offboard (see
  [the dry-run section's RC failsafe checks](#dry-run)).
- A laptop with QGroundControl installed, for firmware flashing and PX4
  parameter work (steps 2, 4, and the dry run) — this is the one piece of
  this whole project that genuinely needs a tool outside Docker, since it
  talks to the Pixhawk directly over USB before any of this repo's own
  software is involved at all.

<a id="flash-px4"></a>

## 2. Flash PX4 to the Pixhawk 6C

This repo is pinned to **PX4 v1.17.0** everywhere (`.env`'s `PX4_VERSION`,
the sim image, `px4_msgs`/`px4_ros_com` branches) — the physical Pixhawk
6C must run the exact same version, or message-definition mismatches
between the firmware and this repo's `px4_msgs` will cause silent DDS
failures (confirmed as a real failure class in sim — see
[Known Issues #6](known-issues.md#issue-6) for the versioned-topic-name
consequence of a version mismatch).

1. Connect the Pixhawk 6C to your laptop via its USB port (USB-C on this
   board — confirm against your specific unit, connector types have
   changed across Pixhawk hardware revisions).
2. Open QGroundControl → **Vehicle Setup → Firmware**.
3. Select **PX4 Flight Stack**, then the **specific version** option (not
   "latest stable") and choose **v1.17.0** explicitly.
4. Flash, wait for it to complete, and let QGroundControl reconnect.
5. Confirm the version: **Vehicle Setup → Firmware** should report
   `v1.17.0`. If PX4 v1.17.0 isn't offered in QGroundControl's version
   list yet (firmware release timing vs. QGC's own list can lag), flash
   from a manually-downloaded `.px4` build of the `v1.17.0` tag from
   PX4's own releases instead — do not substitute a different version
   "close enough."
6. **Airframe**: select the closest matching quadcopter X airframe for a
   generic/custom frame (e.g. Generic Quadcopter). Do NOT select the
   x500-specific airframe — that's this project's SIM airframe (a real,
   specific Holybro frame), and its default parameters assume that exact
   physical vehicle. Getting the airframe selection wrong here is a real,
   documented failure mode in this project's own SITL bring-up — see
   [Known Issues #4](known-issues.md#issue-4)'s NAV_DLL_ACT/battery-check
   story for what happens when an airframe's assumptions don't match the
   actual vehicle (there it was sim-vs-model; here it would be
   airframe-preset-vs-your-actual-frame).
7. **ESC/motor setup, frame geometry, and RC calibration**: standard PX4
   setup, specific to your frame and outside this repo's scope — complete
   these in QGroundControl before continuing. Confirm motor spin
   direction and prop rotation are correct with props OFF before you ever
   touch this repo's software (this project's own motor-direction check
   happens again, with props still off, in [the dry run](#dry-run) —
   that's a deliberate second look, not a duplicate).

<a id="wiring"></a>

## 3. Physical connections: Pixhawk ↔ Orange Pi

**You need TWO independent serial links, not one.** This is a real design
point, not an arbitrary choice: this stack talks to PX4 over uXRCE-DDS
(nearly everything — arm, offboard, telemetry, the whole
`common_control`/`common_missions` stack) but the uXRCE-DDS bridge
**cannot set PX4 parameters** in this pinned version
([Reference §8](reference.md#sec-8);
[Known Issues #14](known-issues.md#issue-14)) — so the one-shot
localization-source switch (`set_localization_source`) talks to PX4 over
a **separate MAVLink connection**. Two different wire protocols need two
different transport links; there is no way to multiplex both PX4 protocol
stacks over a single serial port here.

### Link 1 — uXRCE-DDS, over USB (recommended)

Plug the Pixhawk 6C's own USB port directly into a USB-A/C port on the
Orange Pi with a standard USB data cable. PX4 exposes its uXRCE-DDS
client over this same USB connection by default in recent firmware — no
extra wiring, no voltage-level concerns, and it's the same mechanism this
project's own sim side already uses conceptually (a single reliable
digital link, just USB instead of the sim's loopback UDP). This becomes
your `PIXHAWK_SERIAL_PORT` (commonly `/dev/ttyACM0` on Linux for a native
USB-CDC device like this — **confirm, don't assume**, see
[the dry run](#dry-run)).

### Link 2 — MAVLink, for the localization-source switch only

This link needs its own physical connection, separate from Link 1. Two
options, in order of how strongly this guide recommends them:

**Option A (recommended for a first bring-up): USB-to-TTY serial
adapter.** A cheap 3.3V-logic USB-to-serial adapter (FTDI, CP2102, or
similar — confirm it's a **3.3V** part, not 5V-only, before buying) gives
you a TX/RX/GND breakout on three wires and a plain USB-A plug on the
other end for the Orange Pi's second USB port. Wire the adapter's 3 pins
to the Pixhawk 6C's **TELEM2** port using the pinout below, and plug the
adapter into any spare Orange Pi USB port. This is the safer default
specifically because a wiring mistake here damages a $5 adapter, not
either of your two real boards.

**Option B (advanced): direct GPIO UART wiring**, Orange Pi's 40-pin
header straight to Pixhawk's TELEM2, no adapter in between. Only do this
if you've already confirmed the exact UART TX/RX pin numbers on **your
specific Orange Pi 5 Plus board revision** against its official pinout
diagram — this guide deliberately does NOT give you specific GPIO pin
numbers, because they could not be confirmed with confidence for this
board during authoring, and a wrong pin here (especially one that turns
out to be 5V rather than a UART signal) can damage the Orange Pi.
Consult Orange Pi's own official user manual
(orangepi.net's product page for your board revision) for the
authoritative pin diagram before wiring anything. The Orange Pi's GPIO
header itself IS confirmed 3.3V logic, matching the Pixhawk side, so
voltage compatibility isn't the issue — knowing which physical pin is
which is.

**Pixhawk 6C TELEM2 pinout** (6-pin JST-GH 1.25mm, confirmed against
[Holybro's own docs](https://docs.holybro.com/autopilot/pixhawk-6c/pixhawk-6c-ports)):

| Pin | Signal | Voltage | Wire to |
|---|---|---|---|
| 1 | VCC | +5V | **Leave disconnected** if your adapter/Orange Pi is independently powered (it should be, per [step 1](#what-you-need)) — backfeeding 5V from an unintended second source into either board is a real way to damage something. |
| 2 | UART5_TX (out) | 3.3V | Adapter/GPIO **RX** — TX always wires to RX, never TX to TX. |
| 3 | UART5_RX (in) | 3.3V | Adapter/GPIO **TX** — see above. |
| 4 | UART5_CTS (in) | 3.3V | Leave unconnected unless you've deliberately configured hardware flow control on both ends. |
| 5 | UART5_RTS (out) | 3.3V | Leave unconnected, same reasoning. |
| 6 | GND | GND | Adapter/GPIO **GND** — required; two boards with a data link but no shared ground reference is a real, common source of unreliable serial links. |

**The single most common mistake wiring any UART pair is a TX/RX
crossover error** — Pixhawk TX must reach the OTHER side's RX, and vice
versa, never TX-to-TX. If the link doesn't come up in
[the dry run](#dry-run), swap TX/RX before assuming anything more
complicated is wrong.

Configure the matching PX4 side in QGroundControl → **Parameters**:
- Confirm `UXRCE_DDS_CFG` (or your Pixhawk 6C's equivalent DDS client
  port-select param) points at USB, matching Link 1 above.
- Set `MAV_1_CONFIG` (or the next free `MAV_x_CONFIG` slot) to **TELEM2**,
  with `MAV_1_MODE` set to a mode that streams heartbeats and accepts
  `PARAM_SET` (Normal or Onboard — NOT the read-only "Config" mode used
  for some ground-station-only links).
- Set TELEM2's baud rate to **115200** — `set_localization_source`
  connects via plain pymavlink with no baud override exposed on the
  command line yet, and pymavlink's own default for a serial connection
  string is 115200. Setting the PX4 side to match is simpler than
  patching this repo's Python for a one-shot bring-up script; if you'd
  rather change the code instead, `set_localization_source.py`'s
  `mavlink_connection()` call is the one line to touch.

<a id="px4-config"></a>

## 4. Configure PX4 for GPS flight

1. **Wire the external GPS/compass module** to the Pixhawk 6C's GPS port
   per its own pinout — do this before powering on with props attached.
2. **Compass calibration**: QGroundControl → **Vehicle Setup → Sensors →
   Compass** — the standard rotate-through-all-orientations calibration.
   Do this with the vehicle away from large metal objects/cars/rebar-
   reinforced floors, which is a common source of a compass calibration
   that "passes" but is subtly wrong.
3. **GPS lock**: with a clear sky view, confirm QGroundControl's GPS
   status shows a 3D fix with a reasonable satellite count (a real fix,
   not just "GPS detected") — this gets re-checked as part of
   [the dry run](#dry-run) too, but do a first pass here while you're
   already in QGroundControl.
4. **Arming/safety parameters worth checking explicitly** (PX4 defaults
   are reasonable, but confirm rather than assume, matching this
   project's own "verify live" habit):
   - `COM_ARM_WO_GPS`: should be `0` (disallow arming without a GPS fix)
     for this first bring-up — you want PX4's own prearm checks backing
     you up, not just this repo's software.
   - `NAV_DLL_ACT`/`NAV_RCL_ACT` (datalink-loss / RC-loss failsafe
     actions): this stack flies OFFBOARD with no QGroundControl attached
     during a flight, and no RC command stream either if you're not
     holding a transmitter — decide deliberately what PX4 should do if
     either link drops (Land or RTL are the safe choices for a first
     flight; do not leave a real vehicle's failsafe action unconfigured).
     Sim's SITL airframe override disables these entirely
     ([Known Issues #4](known-issues.md#issue-4)) — that is **SITL-only**
     and must NOT be replicated on real hardware; that override file is
     explicitly scoped to a simulation-only airframe ID and does not
     apply here. You'll actually TEST these failsafes, not just set
     them, in [the dry run](#dry-run).
   - PX4's own **geofence** (`GF_*` parameters, independent of this
     project's own software geofence in `offboard_control_node.py`):
     configure a real PX4-level geofence around your test area too —
     see [step 7](#geofence) for deriving the matching value for this
     project's own software geofence. This project's own geofence
     ([Reference §7](reference.md#sec-7),
     [Known Issues #24](known-issues.md#issue-24)) is real, tested
     defense-in-depth in sim, but it's still one layer — PX4's own
     hardware-level failsafe is a second, independent one worth having
     for a first real flight.

<a id="opi-setup"></a>

## 5. Set up the Orange Pi

1. **Download Ubuntu 22.04 for the Orange Pi 5 Plus specifically** — from
   Orange Pi's own official downloads (orangepi.net's product page for
   the 5 Plus, "Official Tools/OS" section), NOT a generic Ubuntu ARM64
   ISO. Single-board computers need a vendor-provided image with the
   correct kernel/bootloader/device-tree for that exact board — a generic
   image will not boot correctly (wrong kernel = no working peripherals,
   possibly no boot at all).
2. **Flash it** to your microSD/NVMe using `balenaEtcher`, `dd`, or
   Orange Pi's own recommended flashing tool (their download page names
   one) — from another Linux/Mac/Windows machine, not the Orange Pi
   itself (it isn't running anything yet).
3. **First boot**: insert the media, power on. Default login credentials
   are on Orange Pi's own documentation for this image (commonly
   `orangepi`/`orangepi`, but confirm against the image you actually
   downloaded — this varies by release).
4. **Network**: connect Ethernet (simplest for a first bring-up) or
   configure WiFi (`nmtui` is the simplest interactive tool on a fresh
   Ubuntu server image). Confirm you have an IP address:
   ```bash
   ip addr show
   ```
5. **Enable SSH and confirm you can reach it from your laptop** — from
   here on, work over SSH; there's no need to attach a monitor/keyboard
   to the drone itself for anything in this guide.
   ```bash
   ssh orangepi@<the-ip-you-just-found>
   ```
6. **Update the system**:
   ```bash
   sudo apt update && sudo apt full-upgrade -y
   sudo reboot
   ```
7. **Confirm you actually have an ARM64 board booted correctly** before
   going any further — this catches a wrong-image flash immediately
   rather than partway through a long Docker build:
   ```bash
   uname -m        # expect: aarch64
   cat /etc/os-release   # expect: Ubuntu 22.04 (jammy)
   ```
8. **Install Docker** (same install this repo's
   [Setup Guide](setup-guide.md#sec-3) uses on the dev host — nothing
   hardware-specific about this step):
   ```bash
   curl -fsSL https://get.docker.com | bash
   sudo usermod -aG docker $USER
   newgrp docker
   docker --version
   docker compose version
   ```
9. **Clone this repo onto the Orange Pi** (same repo, same clone — this
   project is one codebase for both sim and hardware, see
   [Architecture](../README.md#architecture)):
   ```bash
   git clone https://github.com/abhishekbera86/gps-denied-drone.git
   cd gps-denied-drone
   ```

<a id="build"></a>

## 6. Build the hardware image

```bash
make build-hw
```

Builds `docker/Dockerfile.hw_autonomy` — ARM64-native (no cross-compile),
so this runs **on the Orange Pi itself** and will take a while,
particularly the `librealsense2` from-source build (that Dockerfile's own
header explains why it can't come from apt on ARM64). This is the exact
kind of step this project's own history says to expect real, silent-until-
you-check failures from (see [Known Issues](known-issues.md) generally,
and this Dockerfile's own comments for the specific risk points already
anticipated) — watch the build output, don't assume a clean exit means
everything inside actually works, and file/fix problems as you find them.
GPS-only bring-up doesn't strictly need `librealsense2`/`realsense-ros` to
succeed, but `make build-hw` builds the whole image in one shot — if the
RealSense layers fail and you want to unblock GPS testing immediately,
comment those layers out of the Dockerfile temporarily rather than
reworking the whole build; re-enable them before
[Phase 2](hardware-bringup-vio.md).

Then bring the `hw` profile up (there is no sim equivalent to start here
— `make sim` starts `px4-sim`, an x86_64 simulator container, which has
no purpose on the Orange Pi):

```bash
docker compose --profile hw up -d
make build-ws-hw
```

<a id="geofence"></a>

## 7. Derive a real geofence limit

`hw_params.yaml`'s `geofence_hard_limit_m: 10.0` is an **explicitly
untested placeholder** — not derived from any real test area, unlike
sim's `3.75`, which IS derived from `vio_test.sdf`'s actual fence
position ([Reference §7](reference.md#sec-7)). Before any free flight:

1. Measure the actual usable radius of your test area from the planned
   takeoff point to the nearest real obstacle/boundary.
2. Set `geofence_hard_limit_m` in
   `ros2_ws/src/hw_bringup/config/hw_params.yaml` to
   `(that measured distance) - 1.0m`, matching the safety-margin logic
   sim's own value already uses. Set it in **both** the hover profile's
   section AND `square_mission`'s section — this project has already hit
   the exact mistake of adding a geofence value to only one mission
   section once in sim ([Known Issues #34](known-issues.md#issue-34)'s
   "real bug found while wiring the param" note).
3. This is a config-only edit — no rebuild needed
   ([Setup Guide §4.4](setup-guide.md#sec-4-4)'s reasoning applies
   identically here).

<a id="dry-run"></a>

## 8. Dry run — verify everything before any prop spins

**Remove the props for this entire section.** Nothing below should ever
command meaningful thrust, but the point of a dry run is removing even
the possibility.

### 8.1 Power-on and basic sanity

1. Power the Pixhawk alone first (bench power or battery, your choice) —
   confirm it boots: listen for the startup tune, watch for the LED to
   settle into a steady/blinking pattern rather than a fast error-flash
   (PX4's own docs define its LED/tune codes if you want the specific
   meanings; the practical check here is "no continuous error state").
2. Power the Orange Pi. Confirm no unusual smell, no component
   uncomfortably hot to the touch within the first minute — a basic but
   real check before trusting either board with anything else.
3. SSH into the Orange Pi and confirm `docker ps` shows `hw-autonomy`
   running (from [step 6](#build)).

### 8.2 Confirm both serial links, by device enumeration

Do not assume `/dev/ttyUSB0`/`/dev/ttyACM0` — confirm both, one at a
time:

```bash
ls -la /dev/tty* | grep -E 'ACM|USB'
dmesg | grep -iE 'tty|usb' | tail -30
```

Plug in one link at a time (DDS/USB, then MAVLink/adapter) and re-run
`ls /dev/tty*` between each — the most reliable way to know which device
node is which, rather than guessing from naming convention alone. Once
confirmed, set both in `.env`:

```bash
# .env
PIXHAWK_SERIAL_PORT=/dev/ttyACM0   # whatever you actually confirmed above
PIXHAWK_BAUD_RATE=921600
```

(There's no `.env` entry for the MAVLink link — it's a per-invocation
choice like `LOCALIZATION=`/`MISSION=`, passed as `MAVLINK_URL=` at the
command line, matching this project's own established pattern for
runtime-vs-build-time config — see
[Known Issues #19](known-issues.md#issue-19) for why a value like this
belongs on the command line, not hardcoded into `.env`.)

### 8.3 Confirm the DDS link carries real data

```bash
make shell-hw
source /opt/ros/humble/setup.bash
source /opt/px4_ros2_ws/install/setup.bash
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_status_v1 --once
```

You should see the full `/fmu/in/*`/`/fmu/out/*` topic set, same as sim
([Reference §6](reference.md#sec-6)) — if you see nothing, the uXRCE-DDS
agent isn't reaching PX4; recheck [step 8.2](#dry-run) before going
further. This is the single most likely place for this bring-up's first
real bug to show up — treat "topics exist but look stale/frozen" as a
failure too, not just "topics missing entirely."

### 8.4 Confirm the MAVLink link and the localization-source switch

```bash
ros2 run common_perception set_localization_source \
    --source gps --mavlink-url <your MAVLINK_URL from step 3>
```

Expect `connected (system 1, component 0)` followed by confirmed
`EKF2_GPS_CTRL`/`EKF2_EV_CTRL` parameter sets (GPS mode's values — see
[Reference §8](reference.md#sec-8)). If this hangs on "connecting," the
MAVLink link ([step 3](#wiring)) isn't up — recheck wiring, baud, and
`MAV_1_CONFIG`/`MAV_1_MODE` before assuming a software problem.

### 8.5 Sensor health

1. **GPS**: QGroundControl (or `ros2 topic echo
   /fmu/out/vehicle_gps_position --once` if you'd rather stay in the
   hw-autonomy shell) should show a genuine 3D fix, not just "GPS
   present."
2. **EKF2 convergence**: wait for `pre_flight_checks_pass: true` in
   `vehicle_status_v1`'s output (same field [step 8.3](#dry-run)'s
   `echo` already showed you) — PX4's EKF2 needs real GPS lock and real
   sensor-bias convergence time here, the same
   ["arming rejected at first, this is normal" pattern documented in
   sim](known-issues.md#issue-8), just for real reasons now instead of
   SITL's simulated-time version of the same wait.
3. **Compass**: QGroundControl's Sensors tab should show no large
   deviation/interference warnings at the vehicle's actual resting
   location (not just wherever you calibrated it — nearby metal, wiring,
   or the Orange Pi itself can introduce real interference at the
   assembled vehicle's compass location specifically).

### 8.6 RC link and failsafe checks

Props still off:
1. Confirm the RC transmitter is bound and QGroundControl's **Vehicle
   Setup → Radio** shows live input on every channel you move.
2. **Kill-switch test**: with the vehicle armed (via `make hw-flight-test`
   below, or QGroundControl's own arm button — either way, props off),
   trigger your configured kill switch and confirm it disarms
   immediately. Do this before you ever trust the kill switch in the air.
3. **RC-loss failsafe test**: with the vehicle armed and props off, turn
   the RC transmitter OFF and confirm PX4 responds with the failsafe
   action you configured in [step 4](#px4-config) (watch the mode change
   in QGroundControl, or the corresponding log line) rather than
   continuing to hold whatever it was last commanded — do NOT skip this
   because "it's just a parameter, it must work"; confirming the
   configured behavior actually fires is the entire point of a dry run.

### 8.7 Motor test (PX4's own tool, independent of this repo's software)

QGroundControl → **Vehicle Setup → Motors** (or the **Actuators** tab on
newer QGC) lets you spin each motor individually at low throttle. This
validates motor-to-output mapping and spin direction at the
ESC/hardware level, independent of anything this repo's software
controls — do this BEFORE the offboard arm test below, since it isolates
"is the airframe's motor wiring correct" from "does this repo's software
correctly command the airframe," which are different questions worth
answering separately. Confirm every motor spins the expected direction
for its position before continuing.

### 8.8 This repo's own arm/offboard/land cycle

```bash
make hw-flight-test
```

Props still off. Confirm: PX4 accepts the offboard mode switch, arming
succeeds (`Armed and offboard — climbing to takeoff height` in the log —
same code, same message as sim), then **immediately disarm manually**
(`px4-commander disarm -f` from a second `shell-hw` session, or your RC
kill switch, now confirmed working from [step 8.6](#dry-run)) rather
than letting it try to "climb" with no real thrust — the point here is
confirming the arm/offboard/command path works, not watching the motors
spin uselessly.

### 8.9 Dry-run checklist

Don't proceed to [restrained hover](#restrained-hover) until every line
below is genuinely checked, not assumed:

- [ ] Both boards power on cleanly, no smell/heat concerns
- [ ] `hw-autonomy` container running
- [ ] Both serial device paths confirmed by enumeration, set in `.env`
- [ ] `/fmu/*` topics present and updating (not stale)
- [ ] MAVLink link connects; `set_localization_source --source gps` succeeds
- [ ] GPS 3D fix confirmed
- [ ] `pre_flight_checks_pass: true`
- [ ] Compass clean at the vehicle's actual assembled location
- [ ] RC input live on all channels
- [ ] Kill switch disarms immediately, tested
- [ ] RC-loss failsafe fires the configured action, tested
- [ ] Every motor spins the correct direction (QGC motor test)
- [ ] `make hw-flight-test` arms/offboards/disarms cleanly, props off
- [ ] Real geofence limit derived and set in BOTH `hw_params.yaml` sections ([step 7](#geofence))

<a id="restrained-hover"></a>

## 9. Restrained hover test — props on

Props on, vehicle physically restrained (tethered/held down/on a test
stand rated for this) so it cannot actually leave the ground even at full
commanded thrust. Confirm:
- Motors spin up cleanly on arm, correct rotation directions (re-confirms
  [step 8.7](#dry-run) under real arm/offboard control, not just QGC's
  motor test), no unusual vibration/noise.
- `make hw-flight-test` again — verify the climb command produces
  correct, proportionate thrust response (motors clearly working harder
  as the "climb" setpoint is commanded), not just that arming succeeded.
- Land/disarm cycle completes and disarms cleanly
  (`land_disarm_low_throttle_dwell_s`/`land_disarm_max_timeout_s` fallback
  behavior, [Reference §7](reference.md#sec-7)) — this is exactly the
  mechanism [Known Issues #21](known-issues.md#issue-21) exists to
  guarantee even when PX4's own auto-disarm doesn't fire.

Do not proceed to a free hover until this is completely clean, repeated
at least twice.

<a id="first-hover"></a>

## 10. First free hover

Outdoor, open area, clear of obstacles and people, good GPS visibility,
calm wind. Standard first-flight precautions apply and are not specific
to this stack — RC transmitter in hand with a kill switch you've already
tested working ([step 8.6](#dry-run)), a spotter if possible, and a plan
for what "abort" means before you arm.

```bash
make hw-flight-test
```

`hw_params.yaml`'s hover profile is deliberately conservative for this
first flight: `takeoff_height_m: 1.5` (vs. sim's `2.0`),
`max_velocity_m_s: 0.5` (vs. sim's `1.0`) — see
[Reference §7](reference.md#sec-7) for the full parameter table and why
each hardware value differs from its sim counterpart. Watch the full
arm → climb → hover → land → disarm cycle and confirm it matches sim's
behavior exactly (same code — see [Architecture](../README.md#architecture)
for why that's the whole design point), just with real physics.

<a id="first-mission"></a>

## 11. First GPS mission

Once the hover test is clean and repeatable:

```bash
make hw-mission MISSION=square
```

`hw_params.yaml`'s `square_mission` section flies a smaller footprint
than sim (`side_length_m: 2.0` vs. sim's `3.0`) at a much more
conservative speed (`max_velocity_m_s: 0.1` vs. sim's `0.8`) — deliberate
for a first real mission, not a bug; retune upward only after this is
proven reliable at the conservative settings.

**GPS-only Phase 4 is done once this flies repeatably and reliably.**
Continue to [Phase 2: VIO](hardware-bringup-vio.md) when you're ready to
mount and calibrate the camera — nothing in that phase should touch or
regress anything validated here, since `LOCALIZATION=gps` never loads any
VIO code at all.

<a id="troubleshooting"></a>

## 12. Troubleshooting

- **DDS bridge shows no topics at all**: check the serial device/baud
  from [the dry run](#dry-run) first — a wrong device path is the most
  likely cause, matching sim's own equivalent gotcha
  ([Known Issues #5](known-issues.md#issue-5), though the exact failure
  mode there was container-restart-specific, not applicable here the
  same way).
- **`set_localization_source` can't connect**: confirm the MAVLink link
  from [step 3](#wiring) is wired and configured separately from the DDS
  link, that TX/RX aren't swapped, and that its baud matches (115200
  unless you've patched the code — see step 3).
- **Arming denied, `Preflight Fail: ekf2 missing data`**: same as sim's
  documented normal EKF2-convergence wait
  ([Known Issues #8](known-issues.md#issue-8)) — but if it never clears,
  check GPS lock and compass calibration status in QGroundControl before
  assuming it's this repo's fault.
- **`make build-hw` fails partway**: same advice as sim's equivalent
  ([Known Issues #10](known-issues.md#issue-10)) — Docker layer caching
  means a rerun resumes rather than restarts. If `librealsense2` fails
  specifically and you want to unblock GPS-only testing now, see
  [step 6](#build)'s note on temporarily disabling those layers.
- **RC-loss/kill-switch test didn't do what you configured**: re-check
  `NAV_RCL_ACT`/`NAV_DLL_ACT` and your RC transmitter's own switch
  assignment in QGroundControl — do not proceed to a free flight until
  this behaves exactly as expected on the bench.
- **Something not covered here**: this guide and the underlying
  `hw_bringup`/`hw-autonomy` infrastructure are unverified — expect real
  gaps. When you find and fix one, add it to
  [Known Issues & Fixes](known-issues.md) the same way every sim-side
  bug in this project's history was documented, so the next person (or
  the next session) doesn't rediscover it from scratch.

---

[← Back to README](../README.md) · **Next:** [Phase 2: VIO](hardware-bringup-vio.md)
