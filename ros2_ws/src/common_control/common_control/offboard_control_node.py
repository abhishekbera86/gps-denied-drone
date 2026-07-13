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

import os
from enum import Enum, auto

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
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

    GEOFENCE: bounds are derived automatically from the mission's own route
    (the same waypoints `build_waypoints()` already returns — origin
    included), not a separately hand-maintained box. `geofence_margin_m` is
    added on every horizontal side of that route's bounding box, then
    clamped to `geofence_hard_limit_m` — a physical wall-clearance limit
    independent of any mission's own geometry (see `_geofence_bounds`).
    `geofence_height_margin_m` caps how far above the intended cruise/
    takeoff height the vehicle may climb. This makes the fence "modular" —
    it automatically follows whatever a mission's own `side_length_m`/
    `area_length_m`+`area_width_m`/etc. describe, and needs no code change
    when a mission's geometry params change or a new mission is added.
    Checked in EVERY flying state, including LAND: a breach during
    TAKEOFF/WAYPOINTS/HOVER aborts via `_land()` (the same AUTO_LAND path
    a normal mission end uses); a breach ALREADY during LAND — PX4's own
    AUTO_LAND controller diverging with nothing watching it, confirmed
    live 2026-07-13 flying the real vehicle into the fenced area's wall —
    force-disarms immediately instead, since re-issuing `_land()` at that
    point is a no-op (see `_check_geofence`'s `during_land` docstring).

    ESTIMATE-HEALTH WATCHDOG: a second, independent safety layer alongside
    the geofence, added 2026-07-13 after a VIO pipeline node crashed
    outright mid-flight with nothing watching for it (see
    `_check_estimate_health`). Deliberately source-agnostic — it reads
    PX4's own `xy_valid`/`v_xy_valid` judgement of its current fused
    estimate, not anything vision-specific, so this class stays unaware
    of whether GPS or vision is active (same invariant as everywhere else
    in this project). Same TAKEOFF/WAYPOINTS/HOVER/LAND coverage and the
    same during_land force-disarm-as-last-resort shape as the geofence,
    with one difference: a short dwell (`estimate_invalid_abort_dwell_s`)
    before aborting, since a single-tick validity flicker shouldn't end a
    flight by itself the way a hard position-bound breach should.
    """

    def __init__(self, node_name: str = 'offboard_control_node') -> None:
        super().__init__(node_name)

        self._takeoff_height_m = self._require_param('takeoff_height_m')
        self._hover_seconds = self._require_param('hover_seconds')
        self._waypoint_tolerance_m = self._require_param('waypoint_tolerance_m')
        self._max_velocity_m_s = self._require_param('max_velocity_m_s')
        control_rate_hz = self._require_param('control_rate_hz')
        self._land_disarm_low_throttle_dwell_s = self._require_param(
            'land_disarm_low_throttle_dwell_s')
        self._land_disarm_max_timeout_s = self._require_param(
            'land_disarm_max_timeout_s')
        self._geofence_margin_m = self._require_param('geofence_margin_m')
        self._geofence_height_margin_m = self._require_param('geofence_height_margin_m')
        self._geofence_hard_limit_m = self._require_param('geofence_hard_limit_m')
        self._estimate_invalid_abort_dwell_s = self._require_param(
            'estimate_invalid_abort_dwell_s')

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
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._on_land_detected, PX4_QOS)

        self._vehicle_local_position = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()
        self._vehicle_land_detected = VehicleLandDetected()

        self._state = FlightState.INIT
        self._heartbeat_count = 0
        self._hover_ticks_remaining = 0
        self._control_rate_hz = control_rate_hz

        # Post-landing fallback-disarm bookkeeping (see FlightState.LAND) —
        # `None` means "not currently tracking," set/cleared by
        # `_on_land_detected` and `_land()`.
        self._low_throttle_since = None
        self._land_commanded_time = None
        self._last_disarm_attempt_time = None
        self._logged_land_disarm_timeout_warning = False

        # Geofence breach bookkeeping — TWO independent latches, not one.
        # `_geofence_breached` covers the ordinary TAKEOFF/WAYPOINTS/HOVER
        # abort-to-land path, latched so it only triggers/logs once, not
        # every tick spent descending back through the boundary.
        # `_geofence_land_emergency` covers the SEPARATE during_land
        # force-disarm path — deliberately NOT the same flag: a mission
        # that breaches mid-cruise (setting `_geofence_breached` and
        # transitioning to LAND via the normal path) must still be able to
        # trigger the emergency force-disarm if AUTO_LAND THEN ALSO
        # diverges further during the resulting descent — a real,
        # plausible compounding failure this shares no latch with, or the
        # emergency path would be silently blocked by the earlier, milder
        # breach that led into LAND in the first place.
        self._geofence_breached = False
        self._geofence_land_emergency = False

        # Estimate-health watchdog bookkeeping (2026-07-13) — same
        # two-latch shape as the geofence above, same reasoning: a
        # TAKEOFF/WAYPOINTS/HOVER-phase abort must not block the
        # independent during-land emergency check from also firing if the
        # estimate is STILL invalid (or goes invalid again) after that
        # abort has already switched to LAND. `_estimate_invalid_since` is
        # `None` while the estimate looks healthy, else the time it FIRST
        # went invalid — reset the instant it recovers, so a brief flicker
        # doesn't quietly count toward the dwell threshold much later.
        # See `_check_estimate_health`.
        self._estimate_invalid_since = None
        self._estimate_breach = False
        self._estimate_land_emergency = False

        # NED (x, y, z, yaw) targets flown in order after takeoff; empty
        # means the plain takeoff-hover-land profile.
        self._waypoints: list[tuple[float, float, float, float]] = []
        self._waypoint_index = 0

        self._timer = self.create_timer(1.0 / control_rate_hz, self._control_loop)

        self.get_logger().info(
            f'OffboardControlNode starting: takeoff_height={self._takeoff_height_m}m, '
            f'hover={self._hover_seconds}s, max_velocity={self._max_velocity_m_s}m/s, '
            f'geofence_margin={self._geofence_margin_m}m, '
            f'geofence_height_margin={self._geofence_height_margin_m}m, '
            f'geofence_hard_limit={self._geofence_hard_limit_m}m, '
            f'estimate_invalid_abort_dwell={self._estimate_invalid_abort_dwell_s}s')

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

    def _on_land_detected(self, msg: VehicleLandDetected) -> None:
        self._vehicle_land_detected = msg
        # Tracks how long `has_low_throttle` has been continuously true —
        # NOT the aggregate `landed` flag, which also requires
        # `!horizontal_movement` (position/velocity-estimate-derived, and
        # exactly what can drift post-landing — see FlightState.LAND).
        # `has_low_throttle` is a thrust-SETPOINT comparison in PX4's own
        # land detector (MulticopterLandDetector.cpp), independent of the
        # position/velocity estimate — confirmed by reading that source
        # directly, not assumed. Any sample above the threshold resets the
        # dwell timer to zero, requiring genuinely sustained calm, not a
        # single lucky reading.
        if msg.has_low_throttle:
            if self._low_throttle_since is None:
                self._low_throttle_since = self.get_clock().now()
        else:
            self._low_throttle_since = None

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

    # ── Geofence ─────────────────────────────────────────────────────────
    def _geofence_bounds(self) -> tuple[tuple[float, float], tuple[float, float], float]:
        """((x_min, x_max), (y_min, y_max), z_min) for the current route.

        Derived from the origin plus every queued waypoint (empty for the
        plain hover profile, in which case it's just the origin) — so a
        mission's own geometry params are the ONLY thing that determines
        where the fence sits; nothing here needs to know square vs. survey
        vs. any future mission shape. z is NED (more negative = higher), so
        `z_min` is the CEILING on altitude: `min(cruise/target z values) -
        geofence_height_margin_m`, i.e. how much higher than the highest
        point this route actually visits the vehicle may climb before it's
        considered a breach.

        The horizontal (x/y) box is then CLAMPED to
        `geofence_hard_limit_m` — a physical safety limit independent of
        mission geometry, e.g. "the nearest wall/prop is always at least
        1m past this" (2026-07-13: repeated crashes into the fenced test
        area's actual boundary, one of them because a mission's own
        margin math alone put the soft bound too close to it). Whichever
        of (mission bbox + margin) or (hard limit) is MORE restrictive
        wins — a generously-margined mission can never accidentally widen
        the fence past the known-safe physical limit, and a tightly-
        margined one still gets its own tighter bound if that's smaller.
        """
        xs = [0.0] + [wp[0] for wp in self._waypoints]
        ys = [0.0] + [wp[1] for wp in self._waypoints]
        zs = [self._target_z] + [wp[2] for wp in self._waypoints]
        x_min = max(min(xs) - self._geofence_margin_m, -self._geofence_hard_limit_m)
        x_max = min(max(xs) + self._geofence_margin_m, self._geofence_hard_limit_m)
        y_min = max(min(ys) - self._geofence_margin_m, -self._geofence_hard_limit_m)
        y_max = min(max(ys) + self._geofence_margin_m, self._geofence_hard_limit_m)
        return (
            (x_min, x_max),
            (y_min, y_max),
            min(zs) - self._geofence_height_margin_m,
        )

    def _check_geofence(self, *, during_land: bool = False) -> bool:
        """Abort on a breach; return True once one is latched.

        Deliberately estimate-based (there is no other position source in
        this architecture) and deliberately immediate — no dwell/debounce.
        A false-positive early landing from a noisy sample is an acceptable
        cost; the failure this exists to catch (the vehicle crossing out of
        the fenced/textured area and continuing at speed until it hits the
        ground — reported 2026-07-09) is not something to wait out for
        confirmation.

        `during_land=True` (called from FlightState.LAND) uses a DIFFERENT
        response: `_land()` is a no-op here — PX4's own AUTO_LAND is
        already the active mode, so re-commanding it changes nothing. This
        branch exists because that gap was real: FlightState.LAND was
        previously the ONE flying state with no geofence check at all, and
        a mission that reported "all waypoints reached — landing" then
        drove the real vehicle into the fenced area's actual wall during
        the unmonitored AUTO_LAND descent (2026-07-13) — PX4's own
        landing controller, not this project's code, chasing a diverging
        estimate with nothing watching it. There is no clean way to hand
        AUTO_LAND a corrected trajectory from here, and re-attempting a
        fresh `_land()`/RTL would still be steering off the same corrupted
        estimate that caused the breach — so this is a genuine last resort:
        force-disarm immediately. At this project's low mission altitudes
        (2m) an uncontrolled drop is short and, on the physical evidence
        available, less dangerous than continuing to fly toward/through a
        wall. Ends the flight (`FlightState.DONE`) rather than trying to
        recover — a human should inspect the vehicle after this fires.
        """
        if during_land:
            if self._geofence_land_emergency:
                return True
        elif self._geofence_breached:
            return True
        (x_min, x_max), (y_min, y_max), z_min = self._geofence_bounds()
        pos = self._vehicle_local_position
        if pos.x < x_min or pos.x > x_max or pos.y < y_min or pos.y > y_max or pos.z < z_min:
            if during_land:
                self._geofence_land_emergency = True
                self.get_logger().error(
                    f'GEOFENCE BREACH DURING LANDING: position ({pos.x:.1f}, {pos.y:.1f}, '
                    f'{pos.z:.1f}) is outside x=[{x_min:.1f},{x_max:.1f}] '
                    f'y=[{y_min:.1f},{y_max:.1f}] z>={z_min:.1f} (NED) — AUTO_LAND is '
                    f'flying outside the safe area with no correction possible from here. '
                    f'Force-disarming immediately (last resort).')
                self._disarm(reason='geofence breach during landing')
                self._state = FlightState.DONE
            else:
                self._geofence_breached = True
                self.get_logger().error(
                    f'GEOFENCE BREACH: position ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}) is '
                    f'outside x=[{x_min:.1f},{x_max:.1f}] y=[{y_min:.1f},{y_max:.1f}] '
                    f'z>={z_min:.1f} (NED) — aborting mission, switching to LAND immediately.')
                self._land()
                self._state = FlightState.LAND
            return True
        return False

    def _check_estimate_health(self, *, during_land: bool = False) -> bool:
        """Abort on a SUSTAINED-invalid position/velocity estimate.

        Deliberately does NOT know or care whether GPS or vision is the
        active source — `VehicleLocalPosition.xy_valid`/`v_xy_valid` are
        PX4's OWN internal judgement of whether its current fused estimate
        is trustworthy, already true regardless of which aiding source
        produced it. Reading these instead of, say, the vision pipeline's
        own topics keeps this class unaware of localization source, same
        as everywhere else in this project (`common_perception`'s whole
        package description: "common_control/common_missions never know
        which source is active").

        WHY THIS EXISTS (2026-07-13): confirmed live that a VIO pipeline
        node (`openvins_odometry_bridge`) can crash outright mid-flight —
        an rclpy/DDS message-deserialization RuntimeError under host CPU
        contention, not a logic bug (see that node's own `main()` for the
        full trace) — silently stopping ALL vision data to PX4 for the
        rest of the flight, with nothing watching for it. The mission in
        question happened to still land only mildly drifted, but nothing
        would have caught a worse outcome. This is the general,
        source-agnostic version of that catch: it fires on ANY sustained
        estimate-health problem, not just "the vision bridge process is
        dead" — a stalled DDS agent, a wedged camera bridge, or genuine
        vision divergence bad enough that PX4 itself stops trusting the
        fusion all trigger the same PX4-side flag.

        DWELL, unlike the geofence's deliberately-immediate check: a
        single-tick flicker in `xy_valid` during normal operation (e.g.
        transient EKF2 covariance resets) shouldn't abort a flight by
        itself — `estimate_invalid_abort_dwell_s` requires it to stay
        invalid continuously for that long first. `_estimate_invalid_since`
        resets to `None` the instant validity recovers, so intermittent
        blips can't quietly accumulate toward the threshold.

        Same `during_land` split and reasoning as `_check_geofence`: a
        normal-path abort calls `_land()`; a during-land trigger
        force-disarms immediately, since AUTO_LAND is already the active
        mode and there's nothing to hand it that isn't itself built on the
        same estimate this check just judged untrustworthy.
        """
        if during_land:
            if self._estimate_land_emergency:
                return True
        elif self._estimate_breach:
            return True

        pos = self._vehicle_local_position
        healthy = pos.xy_valid and pos.v_xy_valid
        now = self.get_clock().now()
        if healthy:
            self._estimate_invalid_since = None
            return False

        if self._estimate_invalid_since is None:
            self._estimate_invalid_since = now
        invalid_for_s = (now - self._estimate_invalid_since).nanoseconds / 1e9
        if invalid_for_s < self._estimate_invalid_abort_dwell_s:
            return False

        if during_land:
            self._estimate_land_emergency = True
            self.get_logger().error(
                f'POSITION ESTIMATE INVALID DURING LANDING for {invalid_for_s:.1f}s '
                f'(xy_valid={pos.xy_valid}, v_xy_valid={pos.v_xy_valid}) — AUTO_LAND is '
                f'flying on an estimate PX4 itself no longer trusts, with no correction '
                f'possible from here. Force-disarming immediately (last resort).')
            self._disarm(reason='estimate invalid during landing')
            self._state = FlightState.DONE
        else:
            self._estimate_breach = True
            self.get_logger().error(
                f'POSITION ESTIMATE INVALID for {invalid_for_s:.1f}s '
                f'(xy_valid={pos.xy_valid}, v_xy_valid={pos.v_xy_valid}) — aborting mission, '
                f'switching to LAND immediately.')
            self._land()
            self._state = FlightState.LAND
        return True

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

    def _disarm(self, reason: str = 'post-landing fallback') -> None:
        # param2=21196 is PX4's documented "force" magic value for
        # VEHICLE_CMD_COMPONENT_ARM_DISARM (see Commander.cpp's own CLI
        # handler for `disarm -f`, which sets this same value) — REQUIRED
        # here, confirmed live: PX4's Commander::disarm() rejects a plain
        # disarm with MAV_RESULT_TEMPORARILY_REJECTED unless
        # `vehicle_land_detected.landed` (or `.maybe_landed`) is already
        # true. Both of those are themselves gated on signals affected by
        # the same estimate-drift bug this fallback exists to route around
        # (see FlightState.LAND) — without `force`, PX4 second-guesses this
        # method's own (already more careful, estimate-independent)
        # has_low_throttle-based judgement using the very broken signal it
        # was built to avoid, and the whole fallback would silently no-op.
        # This is not bypassing a safety check; it's substituting this
        # method's own gating (sustained low throttle, checked before ever
        # calling this) for PX4's — the identical mechanism this repo's
        # documented manual escape hatch `px4-commander disarm -f` already
        # uses for exactly this failure mode.
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0, param2=21196.0)
        self.get_logger().info(f'Disarm command sent ({reason}, forced)')

    def _engage_offboard_mode(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Switching to offboard mode')

    def _land(self) -> None:
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._land_commanded_time = self.get_clock().now()
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
            if self._check_geofence() or self._check_estimate_health():
                return
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
            if self._check_geofence() or self._check_estimate_health():
                return
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
            if self._check_geofence() or self._check_estimate_health():
                return
            self._publish_setpoint(0.0, 0.0, self._target_z)
            self._hover_ticks_remaining -= 1
            if self._hover_ticks_remaining <= 0:
                self.get_logger().info('Hover complete — landing')
                self._land()
                self._state = FlightState.LAND
            return

        if self._state == FlightState.LAND:
            # Checked here too (see _check_geofence's during_land docstring)
            # — PX4's own AUTO_LAND is flying this phase, not this node, and
            # it has diverged into the fenced area's actual wall before with
            # nothing watching it. Placed before the disarm check below: an
            # in-progress breach is a bigger emergency than the ordinary
            # landed/not-landed question.
            if (self._check_geofence(during_land=True)
                    or self._check_estimate_health(during_land=True)):
                return

            # Normal path: PX4 auto-disarms on its own once its own land
            # detector confirms landed. Preferred, and — absent the bug
            # below — wins the race against the fallback dwell timer, since
            # PX4 typically auto-disarms within a couple seconds of
            # touchdown, well under the fallback's dwell requirement.
            if self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info('Landed and disarmed — mission complete')
                self._state = FlightState.DONE
                return

            # Fallback: a confirmed real bug (DEVELOPMENT_STATUS.md,
            # "Milestone B result" parts 3-5) can leave PX4's own
            # position/velocity ESTIMATE drifting post-touchdown even
            # though the vehicle is physically at rest, which blocks the
            # normal path above forever (`arming_state` never flips).
            # Two motion-threshold-based fixes were tried and reverted —
            # see part 5's "bigger-picture lesson" — because this
            # project's deliberately slow mission speeds make any
            # velocity/acceleration-magnitude threshold ambiguous between
            # "stopped" and "moving slowly." `has_low_throttle` sidesteps
            # that: it's PX4's own land detector comparing the ACTUATOR
            # thrust SETPOINT against a threshold (confirmed by reading
            # MulticopterLandDetector.cpp directly), not a position/
            # velocity estimate — genuinely independent of the drift bug.
            #
            # Safety-critical framing, not just a convenience filter: this
            # must NEVER disarm based on elapsed time alone (a stuck
            # descent — wind, payload, a mechanical issue — mid-air +
            # blind timeout-disarm would drop the vehicle from height).
            # `land_disarm_max_timeout_s` is a ceiling on how long we WAIT
            # for the real signal, not a trigger by itself — if it's
            # exceeded without `has_low_throttle` ever holding for
            # `land_disarm_low_throttle_dwell_s` straight, this falls back
            # to exactly today's known behavior (stay armed, log loudly,
            # require manual `px4-commander disarm -f`) rather than ever
            # guessing.
            now = self.get_clock().now()
            low_throttle_dwell_s = (
                (now - self._low_throttle_since).nanoseconds / 1e9
                if self._low_throttle_since is not None else 0.0)
            elapsed_since_land_s = (
                (now - self._land_commanded_time).nanoseconds / 1e9
                if self._land_commanded_time is not None else 0.0)
            still_in_auto_land = (
                self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND)

            if (still_in_auto_land
                    and low_throttle_dwell_s >= self._land_disarm_low_throttle_dwell_s):
                if (self._last_disarm_attempt_time is None
                        or (now - self._last_disarm_attempt_time).nanoseconds / 1e9 >= 1.0):
                    if self._last_disarm_attempt_time is None:
                        self.get_logger().warn(
                            'Estimate-independent low-throttle fallback engaged: '
                            f'sustained low throttle for {low_throttle_dwell_s:.1f}s post-'
                            'landing but PX4 has not auto-disarmed (likely the known '
                            'post-landing estimate-drift bug) — issuing explicit disarm.')
                    self._disarm()
                    self._last_disarm_attempt_time = now
            elif (elapsed_since_land_s >= self._land_disarm_max_timeout_s
                    and not self._logged_land_disarm_timeout_warning):
                self._logged_land_disarm_timeout_warning = True
                self.get_logger().error(
                    f'{elapsed_since_land_s:.0f}s since AUTO_LAND commanded with neither '
                    'PX4 auto-disarm nor a sustained low-throttle reading — NOT '
                    'guessing/disarming blindly. Vehicle may genuinely still be '
                    'airborne (wind, payload, mechanical issue) or the low-throttle '
                    'signal itself may be unhealthy. Manual intervention required '
                    "(e.g. 'px4-commander disarm -f' once visually/telemetry-"
                    'confirmed landed).')
            return

        if self._state == FlightState.DONE:
            # Confirmed live (2026-07-09): calling rclpy.shutdown() from
            # inside a timer callback running ON the executor's own spin
            # thread does not return here — it appears to block/deadlock
            # waiting on that same thread, so a `rclpy.shutdown()` +
            # `os._exit(0)` sequence right here never reached the exit
            # call; the process sat alive, still consuming CPU, minutes
            # after "mission complete" was logged. `os._exit(0)` alone
            # makes any graceful rclpy teardown moot anyway — it
            # terminates the OS process immediately regardless of
            # rclpy/executor internal state. See main()'s comment for why
            # this matters beyond tidiness: a launch's sibling VIO/bridge
            # nodes have no exit condition of their own and were observed
            # running an OpenVINS position estimate away 100+ m within
            # minutes when left alive unsupervised after landing.
            os._exit(0)


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

    # rclpy.shutdown() tears down THIS node's ROS context, but confirmed
    # live (2026-07-09) that the OS process can keep running past it —
    # rmw_cyclonedds_cpp (this repo's RMW_IMPLEMENTATION) leaves non-daemon
    # background threads that block a plain interpreter exit. A stuck
    # process here is worse than a normal launch-file leak: this project's
    # `on_exit=Shutdown()` launch handler (autonomy.launch.py) — which
    # exists specifically to tear down the sibling VIO/bridge nodes once
    # the flight finishes — only fires when this process actually exits.
    # Without this, a flight/mission node can log "mission complete" and
    # then sit there indefinitely while OpenVINS keeps running unsupervised
    # (confirmed to run its own position estimate away 100+ m within a
    # couple of minutes — see DEVELOPMENT_STATUS.md). All real cleanup
    # (forced disarm, destroy_node, rclpy.shutdown()) has already happened
    # above by this point, so skipping further Python teardown is safe.
    os._exit(0)


if __name__ == '__main__':
    main()
