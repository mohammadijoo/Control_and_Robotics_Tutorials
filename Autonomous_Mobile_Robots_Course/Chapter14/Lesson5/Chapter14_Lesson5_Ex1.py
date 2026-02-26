#!/usr/bin/env python3
"""Chapter 14 - Lesson 5 (Exercise 1): Health-check checklist for a Nav stack

This script runs a lightweight readiness checklist by querying common Nav2 topics
and TF frames (when ROS 2 is available).

Edit REQUIRED_FRAMES and REQUIRED_TOPICS for your robot stack.

Run (after sourcing ROS 2):
  python3 Chapter14_Lesson5_Ex1.py
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

try:
    from tf2_ros import Buffer, TransformListener
except Exception:
    Buffer = None
    TransformListener = None


REQUIRED_TOPICS = [
    "/tf",
    "/tf_static",
    "/odom",
    "/scan",
    "/amcl_pose",
    "/cmd_vel",
]

REQUIRED_FRAMES = [
    ("map", "odom"),
    ("odom", "base_link"),
    ("base_link", "laser"),
]


class HealthCheck(Node):
    def __init__(self):
        super().__init__("chapter14_lesson5_healthcheck")
        self.buffer = Buffer() if Buffer is not None else None
        self.listener = TransformListener(self.buffer, self) if self.buffer is not None else None

    def topic_ok(self, name: str) -> bool:
        topics = [t[0] for t in self.get_topic_names_and_types()]
        return name in topics

    def tf_ok(self, parent: str, child: str, timeout_s: float = 1.0) -> bool:
        if self.buffer is None:
            return False
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            try:
                self.buffer.lookup_transform(parent, child, rclpy.time.Time())
                return True
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.05)
        return False


def main():
    rclpy.init()
    node = HealthCheck()

    print("\n=== Topic checks ===")
    for t in REQUIRED_TOPICS:
        ok = node.topic_ok(t)
        print(f"{t:20s}: {'OK' if ok else 'MISSING'}")

    print("\n=== TF checks ===")
    for parent, child in REQUIRED_FRAMES:
        ok = node.tf_ok(parent, child, timeout_s=1.5)
        print(f"{parent:10s} -> {child:10s}: {'OK' if ok else 'FAIL'}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
