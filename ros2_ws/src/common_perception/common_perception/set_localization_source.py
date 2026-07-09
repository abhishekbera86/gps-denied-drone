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


def set_localization_source(source: str, mavlink_url: str) -> bool:
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
        for name, value in VISION_LEVER_ARM_FRD.items():
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
    parsed = parser.parse_args(args)

    success = set_localization_source(parsed.source, parsed.mavlink_url)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
