"""
Chapter14_Lesson1.py
Layered navigation architecture demo (Global -> Local -> Reactive safety)
Target: ROS 2 (rclpy), but includes a small non-ROS simulator fallback.

Author: Abolfazl Mohammadijoo (course material)
License: MIT (see repository)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

# -----------------------------
# 1) Minimal geometry utilities
# -----------------------------
@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float  # rad

def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy

# -------------------------------------------------------
# 2) Layer interfaces: global plan, nominal local command
# -------------------------------------------------------
def pick_lookahead_waypoint(path_xy: List[Tuple[float, float]], pose: Pose2D, L: float) -> Tuple[float, float]:
    """
    Return a waypoint at least L meters ahead along the path (very simple).
    """
    if not path_xy:
        return (pose.x, pose.y)

    # Find closest index
    dmin = float("inf")
    imin = 0
    for i, p in enumerate(path_xy):
        d = dist2((pose.x, pose.y), p)
        if d < dmin:
            dmin = d
            imin = i

    # March forward until lookahead reached
    accum = 0.0
    prev = path_xy[imin]
    for j in range(imin + 1, len(path_xy)):
        cur = path_xy[j]
        seg = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        accum += seg
        if accum >= L:
            return cur
        prev = cur
    return path_xy[-1]

def local_nominal_cmd(pose: Pose2D, target_xy: Tuple[float, float], v_ref: float, k_yaw: float) -> Tuple[float, float]:
    """
    A simple local policy: drive forward at v_ref, rotate proportionally to heading error.
    """
    dx = target_xy[0] - pose.x
    dy = target_xy[1] - pose.y
    desired = math.atan2(dy, dx)
    e = wrap_to_pi(desired - pose.yaw)
    w = k_yaw * e
    v = v_ref * max(0.0, math.cos(e))  # slow down if facing away
    return (v, w)

# -------------------------------------------------------
# 3) Reactive safety filter (very lightweight, scan-based)
# -------------------------------------------------------
def safety_filter_scan(u_nom: Tuple[float, float], min_range: float, d_stop: float, d_slow: float) -> Tuple[float, float]:
    """
    Cheap reactive safety:
      - if min_range <= d_stop: stop
      - elif min_range <= d_slow: linearly scale down v and w
      - else: pass-through
    """
    v, w = u_nom
    if min_range <= d_stop:
        return (0.0, 0.0)
    if min_range <= d_slow:
        s = (min_range - d_stop) / (d_slow - d_stop + 1e-9)
        s = max(0.0, min(1.0, s))
        return (s * v, s * w)
    return (v, w)

# -------------------------------------------------------------------
# 4) ROS 2 node skeleton (Global path + Odometry + LaserScan -> cmd_vel)
# -------------------------------------------------------------------
def run_ros2_node() -> None:
    """
    ROS 2 runtime requires:
      sudo apt install ros-<distro>-navigation2 ros-<distro>-nav2-bringup
      pip/apt rclpy, geometry_msgs, nav_msgs, sensor_msgs

    Topics (common):
      - /plan (nav_msgs/Path) from global layer (planner server)
      - /odom (nav_msgs/Odometry)
      - /scan (sensor_msgs/LaserScan)
      - /cmd_vel (geometry_msgs/Twist)
    """
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Path, Odometry
    from sensor_msgs.msg import LaserScan

    class LayeredNavFilter(Node):
        def __init__(self):
            super().__init__("layered_nav_filter")
            self.declare_parameter("lookahead", 0.6)
            self.declare_parameter("v_ref", 0.4)
            self.declare_parameter("k_yaw", 1.8)
            self.declare_parameter("d_stop", 0.35)
            self.declare_parameter("d_slow", 0.9)

            self.path_xy: List[Tuple[float, float]] = []
            self.pose = Pose2D(0.0, 0.0, 0.0)
            self.min_range = float("inf")

            self.sub_path = self.create_subscription(Path, "/plan", self.cb_path, 10)
            self.sub_odom = self.create_subscription(Odometry, "/odom", self.cb_odom, 50)
            self.sub_scan = self.create_subscription(LaserScan, "/scan", self.cb_scan, 50)
            self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 20)

            self.timer = self.create_timer(0.05, self.on_timer)  # 20 Hz local layer

        def cb_path(self, msg: Path):
            self.path_xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.get_logger().info(f"Received path with {len(self.path_xy)} points")

        def cb_odom(self, msg: Odometry):
            self.pose.x = float(msg.pose.pose.position.x)
            self.pose.y = float(msg.pose.pose.position.y)
            q = msg.pose.pose.orientation
            # yaw from quaternion (z-yaw only)
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.pose.yaw = math.atan2(siny, cosy)

        def cb_scan(self, msg: LaserScan):
            # ignore invalid ranges (NaN/inf)
            mr = float("inf")
            for r in msg.ranges:
                if r is None:
                    continue
                if r != r:  # NaN check
                    continue
                if r > 0.0 and r < mr:
                    mr = r
            self.min_range = mr

        def on_timer(self):
            L = float(self.get_parameter("lookahead").value)
            v_ref = float(self.get_parameter("v_ref").value)
            k_yaw = float(self.get_parameter("k_yaw").value)
            d_stop = float(self.get_parameter("d_stop").value)
            d_slow = float(self.get_parameter("d_slow").value)

            target = pick_lookahead_waypoint(self.path_xy, self.pose, L)
            u_nom = local_nominal_cmd(self.pose, target, v_ref=v_ref, k_yaw=k_yaw)
            u_safe = safety_filter_scan(u_nom, self.min_range, d_stop=d_stop, d_slow=d_slow)

            cmd = Twist()
            cmd.linear.x = float(u_safe[0])
            cmd.angular.z = float(u_safe[1])
            self.pub_cmd.publish(cmd)

    rclpy.init()
    node = LayeredNavFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# ----------------------------------------------
# 5) Tiny fallback simulator (no ROS dependency)
# ----------------------------------------------
def run_simple_sim() -> None:
    """
    Demonstrates the architecture without ROS:
      - A pre-defined global polyline path
      - Local nominal command toward a lookahead waypoint
      - Reactive stop if an obstacle is within a synthetic range
    """
    path = [(0.0, 0.0), (2.0, 0.0), (4.0, 1.0), (6.0, 1.0)]
    pose = Pose2D(0.0, -0.2, 0.0)

    # synthetic obstacle at (3.0, 0.2)
    obs = (3.0, 0.2)

    dt = 0.05
    for k in range(400):
        target = pick_lookahead_waypoint(path, pose, L=0.6)
        u_nom = local_nominal_cmd(pose, target, v_ref=0.5, k_yaw=2.0)

        # fake "min_range" as Euclidean distance to obstacle
        min_range = math.hypot(pose.x - obs[0], pose.y - obs[1])
        u = safety_filter_scan(u_nom, min_range, d_stop=0.35, d_slow=1.0)

        v, w = u
        pose.x += v * math.cos(pose.yaw) * dt
        pose.y += v * math.sin(pose.yaw) * dt
        pose.yaw = wrap_to_pi(pose.yaw + w * dt)

        if k % 20 == 0:
            print(f"t={k*dt:5.2f}  pose=({pose.x:5.2f},{pose.y:5.2f},{pose.yaw:5.2f})  range={min_range:4.2f}  cmd=({v:4.2f},{w:4.2f})")

        if math.hypot(pose.x - path[-1][0], pose.y - path[-1][1]) < 0.2:
            print("Reached goal (approx).")
            break

if __name__ == "__main__":
    # Choose one:
    # run_ros2_node()
    run_simple_sim()
