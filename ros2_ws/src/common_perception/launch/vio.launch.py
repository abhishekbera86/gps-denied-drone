"""vio.launch.py — Milestone B: real OpenVINS VIO pipeline, sim side.

Starts four pieces, in order they're declared (no strict sequencing needed
— each is independent until data actually needs to flow between them):
  1. `ros_gz_bridge` (parameter_bridge) — Gazebo camera/IMU -> ROS 2,
     remapped to the exact topic names realsense-ros uses for a real D435i.
     `config/ros_gz_bridge.yaml` is a TEMPLATE, not a literal config — its
     Gazebo-side topic paths are scoped under `/world/<world>/model/
     <model>/...`, and `<world>` in particular is NOT safe to hardcode:
     hardcoding it as `vio_test` is exactly what caused README §12 issue
     35 (the bridge silently subscribed to topics that only exist in one
     specific world, so on any other world it created ROS topics that
     never received a single message, and OpenVINS hung forever with no
     indication why). `_bridge_config_with_world` below renders the
     template with the resolved `world`/`model` launch arguments before
     `parameter_bridge` ever starts, so the bridge always points at
     whichever world is actually running.
  2. OpenVINS (`ov_msckf`'s own `subscribe.launch.py`) — the VIO estimator
     itself, pointed at our derived (not guessed) calibration
     (config/openvins/estimator_config.yaml). `use_stereo`/`max_cameras`
     are passed explicitly here too, matching the YAML's mono settings —
     OpenVINS's launch file also accepts these as separate ROS params, and
     relying on undocumented precedence between the two paths isn't worth
     the risk when they can just agree.
  3. `openvins_odometry_bridge` — republishes OpenVINS's output as
     `/fmu/in/vehicle_visual_odometry`, the same topic the Milestone A
     `loopback_odometry_bridge` uses (drop-in replacement).
  4. `vio_output_check` — a fail-LOUD guard, not part of the VIO pipeline
     itself. Even with (1) no longer hardcoded, `world` still has to
     actually match a world with matching Gazebo topics AND enough visual
     texture for feature tracking (today, that's only `vio_test` —
     confirmed live that the default `empty` world's flat untextured
     ground gives OpenVINS's feature tracker zero usable corners even
     when data IS flowing) — so a wrong or unset `PX4_GZ_WORLD` is still
     possible and still needs to fail loudly rather than hang. Nothing
     before this fails loudly on its own: `set_localization_source` only
     talks to PX4/MAVLink and has no visibility into Gazebo,
     `offboard_control_node` just retries arm/offboard once a second
     forever (by design, for the *normal* 30-60s EKF2-convergence case —
     see README §12 issue 8), so the symptom looks identical to "still
     converging" with no indication anything is actually wrong.

     Deliberately checks OpenVINS's OWN output (`/ov_msckf/odomimu`), not
     its raw camera/IMU input — an earlier version of this check watched
     `/camera/camera/imu` instead, and a live test caught it being wrong:
     once (1) always points the bridge at whatever world is actually
     running, `/camera/camera/imu` gets real data on ANY world, including
     `empty` — an IMU is a physics sensor, indifferent to ground texture.
     Checking the input would have silently stopped catching the
     texture-starvation failure mode the moment the topic-mismatch one
     was fixed. Checking OpenVINS's actual output catches both causes
     (wrong world, or a world with data but not enough texture) with one
     mechanism, because both end in the same observable fact: no valid
     estimate ever comes out. Waits up to 15s (OpenVINS's own static
     initializer needs a few seconds even on a good run — this must not
     false-positive on a merely-slow-but-fine init) and logs a clear
     ERROR naming both possible causes if nothing arrives — it does not
     gate or delay the other three actions (all four start concurrently,
     same as before).

`world` defaults to the `PX4_GZ_WORLD` environment variable — the SAME
one already set on the `make sim`/`make sim-gui` command line to choose
the actually-running Gazebo world (wired through via docker-compose's
`ros2-autonomy.environment`) — so there's exactly one place to set this,
not two that can silently disagree. Override with `world:=<name>` only
if launching this file directly, outside the normal `sim_bringup` flow.

Included by `sim_bringup/launch/sim.launch.py` when `localization_source:=vision`
and Milestone B is selected — see
resource/phase3-gps-denied-localization-source.md.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _bridge_config_with_world(context):
    """Render ros_gz_bridge.yaml's __WORLD__/__MODEL__ template tokens with
    the resolved launch arguments and hand parameter_bridge the result.
    parameter_bridge's `config_file` param is loaded as static YAML with
    no substitution support of its own, so this has to happen before it
    starts — plain string substitution is enough for two tokens, no
    templating engine needed. The rendered file lives under /tmp for the
    life of this container (same pattern OpenVINS's own launch already
    uses for its params files) — nothing else reads it.
    """
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model').perform(context)

    template_path = os.path.join(
        get_package_share_directory('common_perception'), 'config', 'ros_gz_bridge.yaml')
    with open(template_path) as f:
        rendered = f.read().replace('__WORLD__', world).replace('__MODEL__', model)

    rendered_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='_ros_gz_bridge.yaml', delete=False)
    rendered_file.write(rendered)
    rendered_file.close()

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_imu_bridge',
            output='screen',
            parameters=[{'config_file': rendered_file.name}],
        ),
    ]


def generate_launch_description():
    estimator_config = os.path.join(
        get_package_share_directory('common_perception'), 'config', 'openvins',
        'estimator_config.yaml')
    openvins_launch = os.path.join(
        get_package_share_directory('ov_msckf'), 'launch', 'subscribe.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=EnvironmentVariable('PX4_GZ_WORLD', default_value='empty'),
            description=(
                "Gazebo world the camera/IMU bridge's topic paths are scoped "
                "under (/world/<world>/model/<model>/...). Defaults to "
                "PX4_GZ_WORLD (whatever `make sim`/`make sim-gui` was started "
                "with) so this never has to be set separately; must resolve "
                "to a world that's actually running and has enough visual "
                "texture for VIO (today: vio_test — README §4.3/§8).")),
        DeclareLaunchArgument(
            'model', default_value='x500_depth_0',
            description=(
                "PX4's spawned model instance name (its own first-instance "
                "suffix — same regardless of which world is loaded, for a "
                "single-vehicle sim). Re-run `gz topic -l` and update this "
                "default if this project ever spawns more than one vehicle.")),
        OpaqueFunction(function=_bridge_config_with_world),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(openvins_launch),
            launch_arguments={
                'config_path': estimator_config,
                'use_stereo': 'false',
                'max_cameras': '1',
                'rviz_enable': 'false',
            }.items(),
        ),
        Node(
            package='common_perception',
            executable='openvins_odometry_bridge',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['bash', '-c',
                 'timeout 15 ros2 topic echo /ov_msckf/odomimu --once >/dev/null 2>&1 || '
                 'echo "[vio_output_check] ERROR: OpenVINS produced no odometry on '
                 '/ov_msckf/odomimu after 15s -- the position estimate will never '
                 'become valid. Two known causes: (1) PX4_GZ_WORLD does not match '
                 'the world this sim was actually started with (the camera/IMU '
                 'bridge then points at Gazebo topics with no publisher, so '
                 'OpenVINS gets zero input) -- restart with PX4_GZ_WORLD=vio_test '
                 'make sim (or sim-gui); (2) the world has real camera/IMU data but '
                 'not enough visual texture for feature tracking to initialize '
                 '(confirmed live: the empty world'"'"'s flat untextured ground gives '
                 'zero usable corners) -- vio_test (README Secs 4.3/8) is the only '
                 'world confirmed to work today." >&2'],
            name='vio_output_check',
            output='screen',
        ),
    ])
