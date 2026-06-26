#!/usr/bin/env python3
# =============================================================================
# mission.py — Autonomous GPS-Denied Mission (Aerostack2 Python API)
# =============================================================================
# This is the SHARED mission script in quad_core.
# It runs identically in simulation (quad_sim) and on real hardware (quad_real).
# The only difference is which as2_platform_pixhawk config is loaded
# (UDP for sim, serial for real) — this script never knows the difference.
#
# Mission profile:
#   1. ARM (via AS2 — no manual arm needed)
#   2. TAKEOFF to 2.0m AGL
#   3. FLY a 4-point square pattern (4m side, GPS-denied, VIO-based)
#   4. RETURN to launch (hover above origin)
#   5. LAND
#
# Requirements (inside aerostack2 container):
#   source /opt/ros/humble/setup.bash
#   python3 mission.py
# =============================================================================

import rclpy
import sys
import time
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.motion_reference_handler_module import (
    MotionReferenceHandlerModule,
)


# ── Mission Parameters ────────────────────────────────────────────────────
DRONE_NAMESPACE = "drone0"     # Must match world.yaml and as2_platform config
TAKEOFF_HEIGHT  = 2.0          # metres — height above takeoff point
MISSION_SPEED   = 0.5          # m/s — waypoint navigation speed
SQUARE_SIZE     = 4.0          # metres — side length of square pattern
HOVER_TIME      = 3.0          # seconds — pause at each waypoint


# ── Waypoints (ENU frame, relative to takeoff origin) ─────────────────────
# Z = 2.0m (maintain takeoff height throughout)
WAYPOINTS = [
    # (x,   y,    z,    yaw_deg)
    (  SQUARE_SIZE,  0.0,          TAKEOFF_HEIGHT,  0.0),   # East
    (  SQUARE_SIZE,  SQUARE_SIZE,  TAKEOFF_HEIGHT,  90.0),  # NE corner
    (  0.0,          SQUARE_SIZE,  TAKEOFF_HEIGHT,  180.0), # North
    (  0.0,          0.0,          TAKEOFF_HEIGHT,  270.0), # Return to origin
]


def run_mission(drone: DroneInterface) -> bool:
    """Execute the GPS-denied square mission."""

    print("\n" + "="*60)
    print("  GPS-Denied Autonomous Mission — Aerostack2 Python API")
    print("="*60 + "\n")

    # ── 1. ARM ────────────────────────────────────────────────────────────
    print("[1/5] Arming drone...")
    if not drone.arm():
        print("  ERROR: Failed to arm. Check PX4 status and EKF2 health.")
        return False
    print("  ✓ Armed")
    time.sleep(1.0)

    # ── 2. TAKEOFF ────────────────────────────────────────────────────────
    print(f"[2/5] Taking off to {TAKEOFF_HEIGHT}m...")
    if not drone.takeoff(height=TAKEOFF_HEIGHT, speed=0.5):
        print("  ERROR: Takeoff failed.")
        drone.disarm()
        return False
    print(f"  ✓ Reached {TAKEOFF_HEIGHT}m — hovering")
    time.sleep(HOVER_TIME)

    # ── 3. SQUARE PATTERN ─────────────────────────────────────────────────
    print(f"[3/5] Flying {SQUARE_SIZE}m square pattern (GPS-denied, VIO) ...")
    for i, (x, y, z, yaw) in enumerate(WAYPOINTS):
        print(f"  → Waypoint {i+1}/{len(WAYPOINTS)}: x={x:.1f} y={y:.1f} z={z:.1f} yaw={yaw:.0f}°")
        success = drone.go_to.go_to_point_with_yaw(
            x=x, y=y, z=z,
            angle=yaw,
            speed=MISSION_SPEED,
        )
        if not success:
            print(f"  ERROR: Failed to reach waypoint {i+1}.")
            drone.land(speed=0.4)
            return False
        print(f"    ✓ Reached waypoint {i+1}")
        time.sleep(HOVER_TIME)

    print("  ✓ Square pattern complete")

    # ── 4. RETURN TO ORIGIN ───────────────────────────────────────────────
    print("[4/5] Returning to launch origin...")
    drone.go_to.go_to_point_with_yaw(
        x=0.0, y=0.0, z=TAKEOFF_HEIGHT,
        angle=0.0,
        speed=MISSION_SPEED,
    )
    print("  ✓ Over origin — hovering")
    time.sleep(HOVER_TIME)

    # ── 5. LAND ───────────────────────────────────────────────────────────
    print("[5/5] Landing...")
    if not drone.land(speed=0.4):
        print("  WARNING: Land command may not have confirmed.")
    print("  ✓ Landed")

    time.sleep(2.0)
    drone.disarm()
    print("  ✓ Disarmed\n")

    print("="*60)
    print("  Mission complete! ✓")
    print("="*60 + "\n")
    return True


def main():
    rclpy.init()

    drone = DroneInterface(
        drone_id=DRONE_NAMESPACE,
        verbose=False,
        use_sim_time=False,
    )

    try:
        success = run_mission(drone)
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n[!] Mission interrupted by user.")
        print("    Attempting emergency land...")
        drone.land(speed=0.4)
        sys.exit(1)
    finally:
        drone.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
