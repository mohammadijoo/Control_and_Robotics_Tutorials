#!/usr/bin/env python3
"""Chapter 14 - Lesson 5: Lab: Assemble a Full Navigation Stack (ROS 2 Nav2)

This script demonstrates end-to-end integration testing of a navigation stack:
- Wait for Nav2 lifecycle bringup
- Set an initial pose (AMCL)
- Send a NavigateToPose goal (action)
- Log simple metrics (time-to-goal, path length from odom)
- Optionally subscribe to a global plan topic (nav_msgs/Path) if available

Requirements (typical):
- ROS 2 installed and sourced
- nav2_simple_commander installed (Nav2)
- A running Nav2 bringup that exposes /navigate_to_pose and /amcl_pose

Run (after sourcing ROS 2):
  python3 Chapter14_Lesson5.py
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path

try:
    # Nav2 utility wrapper (recommended for quick lab bringups)
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except Exception:
    BasicNavigator = None
    TaskResult = None


@dataclass
class Metrics:
    start_time: float = 0.0
    end_time: float = 0.0
    odom_path_length_m: float = 0.0
    min_goal_dist_m: float = float("inf")
    goal_reached: bool = False


class NavMetricsNode(Node):
    def __init__(self):
        super().__init__("chapter14_lesson5_nav_metrics")

        self.metrics = Metrics()
        self._last_odom_xy: Optional[Tuple[float, float]] = None
        self._goal_xy: Optional[Tuple[float, float]] = None
        self._latest_plan: Optional[Path] = None

        self.create_subscription(Odometry, "/odom", self._on_odom, 50)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 20)
        # Some Nav2 configs publish plan on one of these; keep both optional
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(Path, "/global_plan", self._on_plan, 10)

    def set_goal_xy(self, x: float, y: float) -> None:
        self._goal_xy = (x, y)

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_odom_xy is not None:
            dx = x - self._last_odom_xy[0]
            dy = y - self._last_odom_xy[1]
            self.metrics.odom_path_length_m += math.hypot(dx, dy)
        self._last_odom_xy = (x, y)

        if self._goal_xy is not None:
            d = math.hypot(x - self._goal_xy[0], y - self._goal_xy[1])
            self.metrics.min_goal_dist_m = min(self.metrics.min_goal_dist_m, d)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        # Callback retained for "system is alive" checks; extend if you want covariance stats.
        pass

    def _on_plan(self, msg: Path) -> None:
        self._latest_plan = msg

    def report(self) -> str:
        dt = max(0.0, self.metrics.end_time - self.metrics.start_time)
        return (
            f"Goal reached: {self.metrics.goal_reached}\n"
            f"Time-to-goal (s): {dt:.3f}\n"
            f"Odom path length (m): {self.metrics.odom_path_length_m:.3f}\n"
            f"Closest distance to goal (m): {self.metrics.min_goal_dist_m:.3f}\n"
            f"Plan received: {self._latest_plan is not None}\n"
        )


def make_pose_stamped(node: Node, x: float, y: float, yaw_rad: float, frame_id: str = "map") -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    # yaw -> quaternion (z,w)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


def main():
    if BasicNavigator is None:
        raise RuntimeError(
            "nav2_simple_commander not available. Install Nav2 and nav2_simple_commander, "
            "then source your ROS 2 environment."
        )

    rclpy.init()
    metrics_node = NavMetricsNode()
    navigator = BasicNavigator()

    # 1) Wait for Nav2
    navigator.waitUntilNav2Active(localizer="amcl")

    # 2) Initial pose (set to something valid in your map)
    init_pose = make_pose_stamped(metrics_node, x=0.0, y=0.0, yaw_rad=0.0, frame_id="map")
    navigator.setInitialPose(init_pose)

    # 3) Define goal (edit for your environment)
    goal_x, goal_y, goal_yaw = 2.0, 0.5, 0.0
    goal_pose = make_pose_stamped(metrics_node, goal_x, goal_y, goal_yaw, frame_id="map")
    metrics_node.set_goal_xy(goal_x, goal_y)

    metrics_node.metrics.start_time = time.time()

    # 4) Send goal
    navigator.goToPose(goal_pose)

    # 5) Spin + monitor
    while not navigator.isTaskComplete():
        rclpy.spin_once(metrics_node, timeout_sec=0.1)
        fb = navigator.getFeedback()
        if fb is not None and getattr(fb, "distance_remaining", None) is not None:
            metrics_node.get_logger().info(f"Distance remaining: {fb.distance_remaining:.3f} m")
        time.sleep(0.05)

    metrics_node.metrics.end_time = time.time()
    result = navigator.getResult()
    metrics_node.metrics.goal_reached = (result == TaskResult.SUCCEEDED)

    for _ in range(10):
        rclpy.spin_once(metrics_node, timeout_sec=0.1)

    print("\n=== Chapter14 Lesson5 Metrics Report ===")
    print(metrics_node.report())

    metrics_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
