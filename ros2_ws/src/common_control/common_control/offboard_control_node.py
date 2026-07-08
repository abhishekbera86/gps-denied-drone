#!/usr/bin/env python3
"""OffboardControlNode — arm / takeoff / hover / land over PX4's uXRCE-DDS bridge.

Talks to PX4 directly via px4_msgs — no MAVSDK, no mission framework. Same
node runs unmodified against SITL (Gazebo Harmonic) or a real Pixhawk 6; only
the uXRCE-DDS transport (UDP vs serial) differs, and that's configured
outside this node.

PX4 requires a continuous OffboardControlMode heartbeat (this code publishes
it at 10 Hz) for roughly one second before it will accept the offboard mode
switch — arming or requesting offboard before that stream is established is
rejected. This ordering is the direct analog of the AS2 offboard() bug that
motivated dropping Aerostack2: get the sequence wrong here and PX4 silently
refuses the mode switch.
"""

from enum import Enum, auto

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# PX4 requires this many OffboardControlMode heartbeats to stream before it
# will accept an offboard mode switch request.
OFFBOARD_HEARTBEATS_BEFORE_SWITCH = 10

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class FlightState(Enum):
    INIT = auto()
    TAKEOFF = auto()
    WAYPOINTS = auto()
    HOVER = auto()
    LAND = auto()
    DONE = auto()


class OffboardControlNode(Node):
    """Arm/takeoff/land state machine with an optional waypoint queue.

    With no waypoints set (the default), flies the Phase 1 profile:
    takeoff → hover `hover_seconds` → land. Call `set_waypoints()` before
    takeoff completes (normally right after __init__) to instead fly each
    NED (x, y, z, yaw) target in order after takeoff, then land at the
    final waypoint. Waypoint completion is keyed off position tolerance,
    never elapsed time — the sim can run much slower than wall clock.

    Every tuning value (takeoff height, hover time, control rate, waypoint
    tolerance, max velocity, and — in subclasses — mission geometry) is a
    REQUIRED parameter with no code-side default: it must come from a launch
    `params_file` (see sim_bringup/config/sim_params.yaml or
    hw_bringup/config/hw_params.yaml) or an explicit `-p name:=value`
    override. A missing value fails node startup loudly via `_require_param`
    rather than silently flying with an unintended default.

    `max_velocity_m_s` caps how fast the vehicle cruises toward each
    setpoint (takeoff, hover position, or a waypoint) — see
    `_capped_velocity_toward`. Without it, PX4's own offboard position
    controller accelerates toward every setpoint at its internal velocity
    limit, which is usually much faster than desirable for watching a
    mission fly or for a first real-hardware flight.
    """

    def __init__(self, node_name: str = 'offboard_control_node') -> None:
        super().__init__(node_name)

        self._takeoff_height_m = self._require_param('takeoff_height_m')
        self._hover_seconds = self._require_param('hover_seconds')
        self._waypoint_tolerance_m = self._require_param('waypoint_tolerance_m')
        self._max_velocity_m_s = self._require_param('max_velocity_m_s')
        control_rate_hz = self._require_param('control_rate_hz')

        # NED frame: down is positive z, so climbing means z becomes negative.
        self._target_z = -abs(self._takeoff_height_m)

        self._offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self._trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self._vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._on_local_position, PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._on_vehicle_status, PX4_QOS)

        self._vehicle_local_position = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()

        self._state = FlightState.INIT
        self._heartbeat_count = 0
        self._hover_ticks_remaining = 0
        self._control_rate_hz = control_rate_hz

        # NED (x, y, z, yaw) targets flown in order after takeoff; empty
        # means the plain takeoff-hover-land profile.
        self._waypoints: list[tuple[float, float, float, float]] = []
        self._waypoint_index = 0

        self._timer = self.create_timer(1.0 / control_rate_hz, self._control_loop)

        self.get_logger().info(
            f'OffboardControlNode starting: takeoff_height={self._takeoff_height_m}m, '
            f'hover={self._hover_seconds}s, max_velocity={self._max_velocity_m_s}m/s')

    # ── Parameter loading ────────────────────────────────────────────────
    def _require_param(self, name: str) -> float:
        """Declare and read a required float parameter — no code default.

        Declaring with `Parameter.Type.DOUBLE` (a type, not a value) means
        the parameter has no fallback: with no params_file/-p override,
        rclpy raises `ParameterUninitializedException` the moment we read
        it. We catch that and re-raise with an actionable message — which
        params_file section was expected — so a missing or misspelled
        config entry is a loud, diagnosable startup failure instead of a
        silent wrong-geometry flight.
        """
        self.declare_parameter(name, rclpy.Parameter.Type.DOUBLE)
        try:
            return self.get_parameter(name).value
        except ParameterUninitializedException:
            message = (
                f"Required parameter '{name}' was not provided to node "
                f"'{self.get_name()}'. Launch with a params_file that has a "
                f"'{self.get_name()}:' section setting '{name}' — see "
                f"sim_bringup/config/sim_params.yaml — or pass "
                f"'-p {name}:=<value>' directly."
            )
            self.get_logger().fatal(message)
            raise RuntimeError(message) from None

    # ── Subscribers ──────────────────────────────────────────────────────
    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        self._vehicle_local_position = msg

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        self._vehicle_status = msg

    # ── Command helpers ──────────────────────────────────────────────────
    def _publish_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._offboard_control_mode_pub.publish(msg)

    def _publish_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = self._capped_velocity_toward(x, y, z)
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._trajectory_setpoint_pub.publish(msg)

    def _capped_velocity_toward(self, x: float, y: float, z: float) -> list[float]:
        """Feed-forward velocity toward (x, y, z), capped at max_velocity_m_s.

        Position alone lets PX4's offboard position controller accelerate
        toward the target at its own (much higher) internal velocity limit —
        this feed-forward, combined with the position setpoint, is what
        actually gives max_velocity_m_s control over cruise speed.
        """
        pos = self._vehicle_local_position
        dx, dy, dz = x - pos.x, y - pos.y, z - pos.z
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        if distance < 1e-3:
            return [0.0, 0.0, 0.0]
        scale = self._max_velocity_m_s / distance
        return [dx * scale, dy * scale, dz * scale]

    # ── Waypoint primitive ───────────────────────────────────────────────
    def set_waypoints(self, waypoints: list[tuple[float, float, float, float]]) -> None:
        """Queue NED (x, y, z, yaw_rad) targets to fly after takeoff.

        Remember z is NED: 2 m above ground is z = -2.0.
        """
        self._waypoints = list(waypoints)
        self._waypoint_index = 0

    def _distance_to(self, x: float, y: float, z: float) -> float:
        pos = self._vehicle_local_position
        return ((pos.x - x) ** 2 + (pos.y - y) ** 2 + (pos.z - z) ** 2) ** 0.5

    def _publish_vehicle_command(self, command: int, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._vehicle_command_pub.publish(msg)

    def _arm(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def _engage_offboard_mode(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Switching to offboard mode')

    def _land(self) -> None:
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Switching to land mode')

    # ── State machine ────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        # PX4 requires this heartbeat every cycle, in every state, right up
        # until disarm — a gap is treated as a lost offboard link.
        self._publish_heartbeat()

        if self._state == FlightState.INIT:
            # PX4 treats "offboard signal present" as heartbeat + an actual
            # setpoint stream — without setpoints flowing, the mode switch is
            # rejected. Stream the takeoff target from the start (harmless
            # while disarmed on the ground).
            self._publish_setpoint(0.0, 0.0, self._target_z)
            self._heartbeat_count += 1

            if self._heartbeat_count >= OFFBOARD_HEARTBEATS_BEFORE_SWITCH:
                offboard = (self._vehicle_status.nav_state
                            == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
                armed = self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

                if offboard and armed:
                    self.get_logger().info('Armed and offboard — climbing to takeoff height')
                    self._state = FlightState.TAKEOFF
                    return

                # Command-and-confirm with retry: PX4 may reject the first
                # attempts (signal timing, transient checks) — keep asking
                # once per second until vehicle_status confirms.
                if self._heartbeat_count % int(self._control_rate_hz) == 0:
                    if not offboard:
                        self._engage_offboard_mode()

                    if not armed:
                        self._arm()

                    self.get_logger().info(
                        f'Waiting for offboard+armed (nav_state={self._vehicle_status.nav_state}, '
                        f'arming_state={self._vehicle_status.arming_state})')
            return

        if self._state == FlightState.TAKEOFF:
            self._publish_setpoint(0.0, 0.0, self._target_z)
            if self._vehicle_local_position.z <= self._target_z + 0.2:
                if self._waypoints:
                    self.get_logger().info(
                        f'Reached takeoff height — flying {len(self._waypoints)} waypoints')
                    self._state = FlightState.WAYPOINTS
                else:
                    self.get_logger().info(
                        f'Reached takeoff height — hovering for {self._hover_seconds}s')
                    self._hover_ticks_remaining = int(self._hover_seconds * self._control_rate_hz)
                    self._state = FlightState.HOVER
            return

        if self._state == FlightState.WAYPOINTS:
            x, y, z, yaw = self._waypoints[self._waypoint_index]
            self._publish_setpoint(x, y, z, yaw)
            if self._distance_to(x, y, z) <= self._waypoint_tolerance_m:
                self.get_logger().info(
                    f'Waypoint {self._waypoint_index + 1}/{len(self._waypoints)} reached '
                    f'({x:.1f}, {y:.1f}, {z:.1f})')
                self._waypoint_index += 1
                if self._waypoint_index >= len(self._waypoints):
                    self.get_logger().info('All waypoints reached — landing')
                    self._land()
                    self._state = FlightState.LAND
            return

        if self._state == FlightState.HOVER:
            self._publish_setpoint(0.0, 0.0, self._target_z)
            self._hover_ticks_remaining -= 1
            if self._hover_ticks_remaining <= 0:
                self.get_logger().info('Hover complete — landing')
                self._land()
                self._state = FlightState.LAND
            return

        if self._state == FlightState.LAND:
            if self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info('Landed and disarmed — mission complete')
                self._state = FlightState.DONE
            return

        if self._state == FlightState.DONE:
            self._timer.cancel()
            rclpy.shutdown()
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        # Raised by spin() when the DONE state calls rclpy.shutdown() —
        # this is the normal mission-complete exit path.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
