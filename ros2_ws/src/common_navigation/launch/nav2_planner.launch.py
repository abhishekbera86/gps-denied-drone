"""nav2_planner.launch.py — Nav2 planner-only bring-up (Phase 5 Milestone B).

Starts exactly three things — deliberately NOT nav2_bringup's full stack
(no controller_server, no bt_navigator; see ../config/nav2_planner_params.
yaml's header for why):
  1. planner_server (nav2_planner) — owns its own internal costmap
     (global_costmap, configured obstacle-free/free-space — no depth/
     obstacle data reconciled yet, see DEVELOPMENT_STATUS.md), exposes
     compute_path_to_pose, and publishes the resulting nav_msgs/Path on its
     native /plan topic — the exact topic
     common_control/offboard_control_node.py's NAV_WAYPOINTS state
     subscribes to. No bridge/translation node needed for that hop.
  2. lifecycle_manager — drives planner_server through its
     configure/activate lifecycle transitions; Nav2 nodes power up
     UNCONFIGURED and never plan without this.
  3. goal_relay (common_navigation) — the one unavoidable piece of glue:
     turns a /goal_pose (geometry_msgs/PoseStamped — same topic RViz2's
     "2D Goal Pose" tool and Milestone A's single-goal path both already
     use) into the compute_path_to_pose action call bt_navigator would
     normally make. It never talks to common_control directly.

No map frame / static map->odom transform: the costmap's own global_frame
is set to `odom` directly (nav2_planner_params.yaml) rather than adding a
separate `map` frame with an identity transform to it — this project has no
absolute/global localization or loop closure (VIO-only), so `map` would
just be a second name for the same frame `odom` already is. Skipping it
entirely is the more honest model of "we have no global correction," not a
missing piece.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('common_navigation'), 'config', 'nav2_planner_params.yaml')

    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='common_navigation',
            executable='goal_relay',
            output='screen',
        ),
    ])
