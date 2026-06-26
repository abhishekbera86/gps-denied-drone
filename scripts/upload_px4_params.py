#!/usr/bin/env python3
# =============================================================================
# upload_px4_params.py — Upload PX4 EKF2 VIO Parameters via MAVLink
# =============================================================================
# Connects to the running PX4 (SITL or real) via MAVLink UDP/serial
# and uploads parameters from the .params file.
#
# Usage (inside aerostack2 or hw_stack container):
#   python3 /scripts/upload_px4_params.py \
#       --params /ros2_ws/src/quad_core/config/ekf2_vio.params \
#       --port 14550
#
# The .params file format: <PARAM_NAME>,<VALUE> (one per line)
# Lines starting with # are comments.
# =============================================================================

import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not found. Install with: pip3 install pymavlink")
    sys.exit(1)


def parse_params(params_file: str) -> dict:
    """Parse a PX4 .params file into a {name: value} dict."""
    params = {}
    with open(params_file, "r") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "," not in line:
                print(f"  [SKIP] Line {line_num}: invalid format: {line!r}")
                continue
            parts = line.split(",", 1)
            name = parts[0].strip()
            try:
                value = float(parts[1].strip())
                params[name] = value
            except ValueError:
                print(f"  [SKIP] Line {line_num}: non-numeric value: {line!r}")
    return params


def upload_params(master, params: dict) -> int:
    """Upload parameters to PX4 via MAVLink. Returns number of successful sets."""
    success = 0
    total = len(params)
    print(f"\n  Uploading {total} parameters...\n")

    for i, (name, value) in enumerate(params.items(), 1):
        print(f"  [{i:3d}/{total}] {name:<30s} = {value}", end="  ")
        try:
            master.param_set_send(name.encode(), value)
            # Wait for ACK (PARAM_VALUE message)
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=3.0)
            if msg:
                print("✓")
                success += 1
            else:
                print("✗  (timeout — no ACK)")
        except Exception as e:
            print(f"✗  ({e})")
        time.sleep(0.05)  # Small delay to not flood the MAVLink connection

    return success


def main():
    parser = argparse.ArgumentParser(
        description="Upload PX4 EKF2 VIO parameters via MAVLink"
    )
    parser.add_argument(
        "--params",
        default="/ros2_ws/src/quad_core/config/ekf2_vio.params",
        help="Path to .params file",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=14550,
        help="MAVLink UDP port (default: 14550 for QGC/PX4 SITL)",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="MAVLink host (default: 127.0.0.1)",
    )
    args = parser.parse_args()

    # ── Parse parameter file ──────────────────────────────────────────────
    print(f"\n==> Loading parameters from: {args.params}")
    params = parse_params(args.params)
    if not params:
        print("  ERROR: No parameters parsed from file.")
        sys.exit(1)
    print(f"  Loaded {len(params)} parameters")

    # ── Connect to PX4 ───────────────────────────────────────────────────
    conn_str = f"udp:{args.host}:{args.port}"
    print(f"\n==> Connecting to PX4 at {conn_str} ...")
    try:
        master = mavutil.mavlink_connection(conn_str)
        master.wait_heartbeat(timeout=10.0)
        print(f"  ✓ Connected to system {master.target_system}, "
              f"component {master.target_component}")
    except Exception as e:
        print(f"  ERROR: Could not connect — {e}")
        print("  Is PX4 running? Try: docker logs px4_sitl | tail -5")
        sys.exit(1)

    # ── Upload ────────────────────────────────────────────────────────────
    n_ok = upload_params(master, params)

    # ── Summary ──────────────────────────────────────────────────────────
    print(f"\n  {'='*50}")
    print(f"  Result: {n_ok}/{len(params)} parameters uploaded successfully")
    if n_ok < len(params):
        print(f"  WARNING: {len(params) - n_ok} parameters failed.")
        print("  Check PX4 parameter names against your firmware version.")
    else:
        print("  ✓ All parameters uploaded. Reboot PX4 to apply.")
        print("  In PX4 shell: param save && reboot")
    print(f"  {'='*50}\n")

    master.close()
    sys.exit(0 if n_ok == len(params) else 1)


if __name__ == "__main__":
    main()
