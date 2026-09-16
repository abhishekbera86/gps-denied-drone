#!/usr/bin/env python3
"""goal_relay — turns a /goal_pose into a Nav2 ComputePathToPose call.

The one piece of glue this milestone needs: with bt_navigator deliberately
excluded (see ../config/nav2_planner_params.yaml's header for why — this
project's own velocity control and safety watchdogs already own vehicle
control, Nav2 is planning-only here), nothing else turns a goal topic into
an action call. This node does nothing else — it never talks to
common_control directly. The path nav2_planner_server computes is published
on its own native /plan topic, which is what
common_control/offboard_control_node.py's NAV_WAYPOINTS state actually
subscribes to (see that file's docstring) — no bridge/translation node for
that hop, matching this project's existing written intent ("Nav2 output
feeds common_control as plain ROS 2 topics").
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalRelay(Node):

    def __init__(self) -> None:
        super().__init__('goal_relay')
        self._client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal_pose, 10)
        self.get_logger().info('goal_relay ready — waiting for /goal_pose')

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'compute_path_to_pose action server not available — is '
                'planner_server active? (ros2 lifecycle get /planner_server)')
            return
        goal = ComputePathToPose.Goal()
        goal.goal = msg
        # `start` deliberately left unset — ComputePathToPose reads the
        # robot's current pose from the costmap's own TF buffer when no
        # start is given, so this node needs no TF lookup of its own.
        self._client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('compute_path_to_pose goal rejected')
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result()
        num_poses = len(result.result.path.poses)
        self.get_logger().info(
            f'compute_path_to_pose finished: status={result.status}, '
            f'{num_poses} pose(s) (published on /plan by planner_server)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
