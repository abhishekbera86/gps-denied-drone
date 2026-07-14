#!/usr/bin/env python3
"""set_localization_source — one-shot switch between GPS and vision (VIO).

Chooses PX4's EKF2 position/velocity fusion source ONCE, before a mission
starts — not a live in-flight switch. Run before the mission node, exits
when done:

    set_localization_source --source gps|vision --mavlink-url udpin:0.0.0.0:14540

PX4 SITL's "offboard" MAVLink instance (px4-rc.mavlink) LISTENS on its own
local port and SENDS unsolicited to port 14540 without waiting to be spoken
to first — so this side must be a UDP *listener* (`udpin:`), not a client
connecting out (`udpout:`/plain `udp:`). Verified live against running SITL.

WHY MAVLINK, NOT THE EXISTING uXRCE-DDS BRIDGE: this project talks to PX4
over its native ROS 2 / uXRCE-DDS bridge everywhere else, but that bridge
does not carry PX4's internal parameter-set protocol in this pinned version
(no `Parameter*` topic is bridged — checked PX4 v1.17.0's dds_topics.yaml
directly). Setting `EKF2_GPS_CTRL`/`EKF2_EV_CTRL` therefore needs a small,
separate MAVLink PARAM_SET side-channel — PX4 SITL already exposes a MAVLink
UDP port independently of the DDS agent, so this needs no PX4 rebuild.

WHY ONLY THESE TWO PARAMS: `EKF2_HGT_REF` (which height source is primary)
requires a reboot to take effect, but we never touch it. Disabling GPS
(EKF2_GPS_CTRL=0) makes PX4's own automatic height-source fallback pick the
next enabled source — baro, on by default — with no reboot needed. Vision
only ever supplies horizontal position + velocity here (HPOS+VEL, bits 0+2);
yaw is left to the magnetometer/gyro to avoid vision-yaw drift, and height
stays on baro. This is the standard PX4 pattern for indoor GPS-denied flight.

EKF2_EV_POS_X/Y/Z (vision-source switch only, set unconditionally alongside
EKF2_EV_CTRL): the vision-sensor lever arm — the published
`/fmu/in/vehicle_visual_odometry` represents the D435i's OWN onboard IMU
(OpenVINS's state, co-located with the color sensor on the `d435i` model —
see docker/px4_sitl_models/d435i/model.sdf), NOT the flight controller's
IMU at base_link. Those are physically offset by the CameraJoint mount
pose (docker/px4_sitl_models/x500_d435i_depth/model.sdf: 0.12/0.03/0.242 m
forward/left/up in the SDF's FLU-like body convention). Without telling
EKF2 about this offset, it silently assumes the vision measurement
originates at its own IMU — wrong by that same lever arm — so any attitude
change (pitch/roll) during flight makes the offset sensor translate through
space in a way the vehicle's own body center does not, and EKF2
misattributes that lever-arm motion as genuine translation. This was a
real, previously-missing configuration gap (confirmed via `grep`: nothing
in this repo ever set these), flagged as a contributing factor to the
AUTO_LAND divergence incident in DEVELOPMENT_STATUS.md "Milestone B result,
part 6/7" — attitude changes are more pronounced during landing/correction
maneuvers than steady cruise, where the mismatch would bite hardest.
PX4's own param docs (params_external_vision.yaml): "position of VI sensor
focal point in body frame" — FRD, so the FLU mount offset above converts
as (X_frd, Y_frd, Z_frd) = (X_flu, -Y_flu, -Z_flu) = (0.12, -0.03, -0.242).

EKF2_EVP_NOISE/EKF2_EVV_NOISE (vision-source switch only, set
unconditionally alongside EKF2_EV_CTRL): confirmed via PX4's own
params_external_vision.yaml that EKF2_EV_NOISE_MD (left at PX4's default,
0, NOT changed here — see below) takes vision measurement noise from the
message itself, "the EV noise parameters are used as a lower bound."
`openvins_odometry_bridge.py` forwards OpenVINS's own raw per-message
covariance untouched, so under the default floor (PX4's own 0.1 m /
0.1 m/s), EKF2's trust in vision can still swing up toward however
confident OpenVINS's covariance estimate feels on a given frame —
plausibly a real contributor to inconsistent flight behavior (accurate on
one run, drifting/wall-hitting on another, nothing else changed) reported
after Phase 3 Milestone B: `resource/Vio_Drift_analysis.txt`. Raising the
floor to 0.3 m / 0.15 m/s bounds how confident EKF2 will EVER treat vision
as being, without disabling the message-based path entirely. Starting
values, not derived from first principles — retune from live results.

TRIED AND REVERTED (2026-07-10): `EKF2_EV_NOISE_MD=1` ("use ONLY the fixed
params, ignore the message's covariance entirely") looked like the more
thorough fix and was tried first — confirmed LIVE to be at minimum
correlated with a regression: two clean, fully-restarted (`docker compose
down`/`up`, not a partial restart) flight attempts both stuck at
`arming_state=STANDBY` for the full test duration, PX4 repeatedly logging
"Arming denied: Resolve system health failures first" with an earlier
"Preflight Fail: ekf2 missing data" ("waiting for estimator to
initialize" — `EstimatorCheck.cpp`). A third attempt, identical except
reverting only this one param back to PX4's default (0), armed, flew the
full `square` mission, and disarmed cleanly, so it was reverted here.

IMPORTANT CAVEAT (2026-07-10, found testing the floor-only fix above): the
exact same "ekf2 missing data"/never-arms symptom ALSO occurred once, on a
clean fully-restarted attempt, WITH EKF2_EV_NOISE_MD left at 0 (this
file's current, "safe" config) — 3 of 4 total clean attempts on this
config armed/flew/landed normally, 1 did not, in the exact same way as the
reverted mode=1 attempts. This means EKF2_EV_NOISE_MD=1 was probably NOT
uniquely responsible for the earlier failures — reverting it was still the
right conservative call (it's not proven safe either), but there was a
SEPARATE, pre-existing flakiness in how reliably EKF2/OpenVINS reach
"estimator initialized" before arming, independent of this file's params.

ROOT-CAUSED (2026-07-10, later same day): NOT a code bug anywhere in this
project, PX4, or OpenVINS — a SITL performance/timing issue. OpenVINS's
own `ROS2Visualizer::visualize_odometry()` (ov_msckf source) refuses to
publish ANY odometry until `(timestamp - _app->initialized_time()) >= 1`
— one full second of OpenVINS's own internal (Gazebo-simulation-derived)
time since init. Confirmed live: `gz topic -e -t
/world/vio_test/stats` measured `real_time_factor: 0.023` (~1/44th of
real time) during a stuck attempt, and `/fmu/in/vehicle_visual_odometry`
received ZERO messages across a full 250-second wall-clock test — `gz
sim` alone was using 448% CPU on an 8-core host with load average >5.
At that RTF, OpenVINS's mandatory 1-second (sim-time) gate needs ~44
real seconds just to START publishing, and can take arbitrarily longer if
RTF degrades further under load — easily exceeding any reasonable
arm-retry patience. This is 100% specific to SITL's Gazebo-lockstep
simulated clock (confirmed via PX4 subscribing to `/world/<world>/clock`
— `px4-rc.gzsim`) and CANNOT recur on Phase 4 real hardware, which has no
simulated clock at all. See README §12 issue 29 for the full
investigation and mitigation options (reduce `vio_test.sdf` prop count/
camera resolution, ensure an idle host before testing, be patient rather
than assuming a stall — this is slowness, not a hang).

EKF2_NOAID_TOUT (vision only, added 2026-07-13, related to but DISTINCT
from the RTF finding above): with `/dev/dri` fixed on master (README §12
issue 31 — headless `px4-sim` now genuinely holds real_time_factor 1.0,
not the ~0.02 it silently ran at before), the OpenVINS 1-second-sim-time
gate above now only costs ~1 REAL second, not 44 — but that exposed a
DIFFERENT, previously-masked tightness: PX4's default
`EKF2_NOAID_TOUT` (5s) is the ENTIRE real-time budget for the whole
vision pipeline to go from "GPS just switched off" to "first vision
estimate PX4 actually accepts" — the MAVLink switch, spawning
`parameter_bridge`/`run_subscribe_msckf`/`openvins_odometry_bridge` as
real OS processes, DDS discovery, first camera frames, OpenVINS's own
init, AND that init's 1-second gate above, all inside 5 real seconds.
Confirmed live: a `square` mission's estimate went `xy_valid=false`
within ~2s of reaching TAKEOFF (this file's own MAVLink switch to
`vision` already consumes real time before the mission node even starts)
— caught cleanly by the new estimate-health watchdog
(`offboard_control_node.py`) rather than flying on a bad estimate, but
the mission never got a real chance to fly. This project's
`dev/camera-tilt` branch hit the identical mechanism and fixed it the
same way: raise `EKF2_NOAID_TOUT` to PX4's own allowed maximum (10s) for
vision mode only — GPS mode's default (5s) is untouched, since GPS's own
aiding is already available at boot with no comparable pipeline-spinup
latency.
"""

import argparse
import struct
import sys

from pymavlink import mavutil

# Bit 0 = horizontal position, bit 1 = vertical position, bit 2 = velocity,
# bit 3 = yaw (EKF2_GPS_CTRL / EKF2_EV_CTRL bitmasks — see EKF/common.h).
SOURCES = {
    'gps': {
        'EKF2_GPS_CTRL': 7,  # HPOS + VPOS + VEL (PX4's own default)
        'EKF2_EV_CTRL': 0,   # vision fusion off
    },
    'vision': {
        'EKF2_GPS_CTRL': 0,  # GPS fusion off — height falls back to baro
        'EKF2_EV_CTRL': 5,   # HPOS + VEL from vision; yaw stays on mag/gyro
        # PX4's own allowed MAXIMUM (module.yaml: max: 10000000) — see
        # this module's docstring's "EKF2_NOAID_TOUT" section for why the
        # 5s default is too tight for the real-time vision-pipeline
        # spin-up cost now that RTF is genuinely 1.0 on this host.
        'EKF2_NOAID_TOUT': 10000000,
    },
}

# Vision sensor lever arm (FRD body frame, meters) — see module docstring.
# Set only when switching to 'vision': GPS mode fuses no vision data, so the
# offset is irrelevant there, and leaving it untouched avoids any risk to
# the GPS path.
VISION_LEVER_ARM_FRD = {
    'EKF2_EV_POS_X': 0.12,
    'EKF2_EV_POS_Y': -0.03,
    'EKF2_EV_POS_Z': -0.242,
}

# Vision fusion noise FLOOR (see module docstring) — set only when switching
# to 'vision', same reasoning as VISION_LEVER_ARM_FRD above. Deliberately
# does NOT touch EKF2_EV_NOISE_MD (left at PX4's default, 0): confirmed
# live that EKF2_EV_NOISE_MD=1 breaks arming entirely (see module
# docstring's "TRIED AND REVERTED"), so these values raise the LOWER BOUND
# PX4 applies on top of OpenVINS's own per-message covariance rather than
# replacing it outright.
VISION_NOISE_VALUES = {
    'EKF2_EVP_NOISE': 0.3,   # m — starting point, retune from live results
    'EKF2_EVV_NOISE': 0.15,  # m/s
}

CONNECT_TIMEOUT_S = 30.0
PARAM_SET_TIMEOUT_S = 5.0


def _log(message: str) -> None:
    print(f'[set_localization_source] {message}', flush=True)


def _int32_to_mavlink_float(value: int) -> float:
    """Encode an int32 PX4 param value into MAVLink's float32 wire slot.

    MAVLink's PARAM_SET/PARAM_VALUE messages carry every param value in a
    single float32 field, regardless of the param's real type. For a PX4
    INT32 param (param_type=MAV_PARAM_TYPE_INT32, e.g. EKF2_GPS_CTRL), PX4
    expects/returns the int32's raw BITS reinterpreted as a float32 — a
    numeric cast (e.g. `float(7)`) is a different, wrong 4 bytes. Verified
    live against running SITL: a naive numeric cast reads back as
    ~9.8e-45, not the actual integer value.
    """
    return struct.unpack('<f', struct.pack('<i', value))[0]


def _mavlink_float_to_int32(value: float) -> int:
    """Inverse of `_int32_to_mavlink_float` — decode a PARAM_VALUE reply."""
    return struct.unpack('<i', struct.pack('<f', value))[0]


def _set_param(conn, name: str, value: int) -> bool:
    """Send PARAM_SET and confirm PX4 accepted it. Returns success."""
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        name.encode('utf-8'), _int32_to_mavlink_float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32)

    ack = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=PARAM_SET_TIMEOUT_S)
    if ack is None:
        _log(f'ERROR: no PARAM_VALUE ack for {name} within {PARAM_SET_TIMEOUT_S}s')
        return False

    ack_name = ack.param_id.rstrip('\x00') if isinstance(ack.param_id, str) else ack.param_id
    ack_value = _mavlink_float_to_int32(ack.param_value)
    if ack_name != name or ack_value != value:
        _log(f'ERROR: {name} confirm mismatch — asked {value}, PX4 reports '
             f'{ack_name}={ack_value}')
        return False

    _log(f'{name} = {value} (confirmed)')
    return True


def _set_float_param(conn, name: str, value: float) -> bool:
    """Send PARAM_SET for a genuine FLOAT param (e.g. EKF2_EV_POS_X).

    Unlike `_set_param`, no int32 bit-reinterpretation — MAVLink's
    PARAM_SET float32 field already IS this value for a real float param.
    Confirmation uses an approximate comparison since the value round-trips
    through a float32 wire format.
    """
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        name.encode('utf-8'), value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    ack = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=PARAM_SET_TIMEOUT_S)
    if ack is None:
        _log(f'ERROR: no PARAM_VALUE ack for {name} within {PARAM_SET_TIMEOUT_S}s')
        return False

    ack_name = ack.param_id.rstrip('\x00') if isinstance(ack.param_id, str) else ack.param_id
    if ack_name != name or abs(ack.param_value - value) > 1e-4:
        _log(f'ERROR: {name} confirm mismatch — asked {value}, PX4 reports '
             f'{ack_name}={ack.param_value}')
        return False

    _log(f'{name} = {value} (confirmed)')
    return True


def set_localization_source(source: str, mavlink_url: str, lever_arm_frd: dict = None) -> bool:
    if source not in SOURCES:
        _log(f"ERROR: unknown source '{source}' — choose from {list(SOURCES)}")
        return False

    _log(f"connecting to PX4 over MAVLink at '{mavlink_url}'...")
    conn = mavutil.mavlink_connection(mavlink_url)
    if conn.wait_heartbeat(timeout=CONNECT_TIMEOUT_S) is None:
        _log(f'ERROR: no MAVLink heartbeat from PX4 within {CONNECT_TIMEOUT_S}s')
        return False
    _log(f'connected (system {conn.target_system}, component {conn.target_component})')

    ok = True
    for name, value in SOURCES[source].items():
        ok = _set_param(conn, name, value) and ok

    if source == 'vision':
        # Airframe-specific physical geometry — NOT the same value on every
        # rig. Defaults to the sim d435i model's own (co-located,
        # near-zero) offset; callers with a different physical mount (e.g.
        # hw_bringup's real D435i on a real airframe — see
        # resource/hardware-bringup-vio.md) must pass the real measured
        # values here rather than relying on the sim default silently
        # applying to different hardware.
        for name, value in (lever_arm_frd or VISION_LEVER_ARM_FRD).items():
            ok = _set_float_param(conn, name, value) and ok
        for name, value in VISION_NOISE_VALUES.items():
            ok = _set_float_param(conn, name, value) and ok

    if ok:
        _log(f"localization source set to '{source}'")
    else:
        _log(f"FAILED to fully set localization source '{source}' — see errors above")
    return ok


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--source', required=True, choices=sorted(SOURCES),
        help='Localization source to activate')
    parser.add_argument(
        '--mavlink-url', required=True,
        help='pymavlink connection string, e.g. udpin:0.0.0.0:14540')
    parser.add_argument(
        '--ev-pos-x', type=float, default=None,
        help=('Vision sensor lever arm X (FRD body frame, meters) — overrides the '
              'sim d435i default. Only meaningful with --source vision. All three '
              '--ev-pos-* must be given together or not at all.'))
    parser.add_argument(
        '--ev-pos-y', type=float, default=None,
        help='Vision sensor lever arm Y (FRD body frame, meters) — see --ev-pos-x.')
    parser.add_argument(
        '--ev-pos-z', type=float, default=None,
        help='Vision sensor lever arm Z (FRD body frame, meters) — see --ev-pos-x.')
    parsed = parser.parse_args(args)

    overrides = [parsed.ev_pos_x, parsed.ev_pos_y, parsed.ev_pos_z]
    if any(v is not None for v in overrides) and not all(v is not None for v in overrides):
        parser.error('--ev-pos-x/--ev-pos-y/--ev-pos-z must be given together, not partially')
    lever_arm_frd = None
    if all(v is not None for v in overrides):
        lever_arm_frd = {
            'EKF2_EV_POS_X': parsed.ev_pos_x,
            'EKF2_EV_POS_Y': parsed.ev_pos_y,
            'EKF2_EV_POS_Z': parsed.ev_pos_z,
        }

    success = set_localization_source(parsed.source, parsed.mavlink_url, lever_arm_frd)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
