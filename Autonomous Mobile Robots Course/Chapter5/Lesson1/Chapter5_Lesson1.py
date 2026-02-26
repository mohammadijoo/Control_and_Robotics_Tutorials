"""
Chapter5_Lesson1.py
Wheel Odometry Computation — Differential Drive (planar SE(2))

This module implements a numerically stable wheel-encoder odometry integrator:
- encoder tick increments -> wheel angle increments
- wheel angle increments -> wheel arc lengths
- wheel arc lengths -> robot pose increment (x, y, theta)
- exact integration for constant curvature over each sample

Dependencies: numpy (optional; used only in demo)
"""

from __future__ import annotations
from dataclasses import dataclass
from math import pi, sin, cos, atan2, fmod, sqrt


def normalize_angle(theta: float) -> float:
    """
    Wrap angle to (-pi, pi].
    """
    # robust wrap using atan2
    return atan2(sin(theta), cos(theta))


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float  # yaw [rad], measured from +x toward +y (right-handed)


@dataclass
class DiffDriveParams:
    r_l: float                 # left wheel radius [m]
    r_r: float                 # right wheel radius [m]
    b: float                   # wheel baseline (track width) [m]
    ticks_per_rev: int         # encoder ticks per motor shaft revolution
    gear_ratio: float = 1.0    # motor rev per wheel rev (>=1 if geared down)


class DifferentialDriveOdometry:
    """
    Differential-drive wheel odometry integrator.
    """
    def __init__(self, params: DiffDriveParams, pose0: Pose2D | None = None) -> None:
        self.params = params
        self.pose = pose0 if pose0 is not None else Pose2D(0.0, 0.0, 0.0)

    def ticks_to_wheel_angle(self, dN: int) -> float:
        """
        Convert incremental encoder ticks to incremental wheel angle [rad].
        """
        denom = float(self.params.ticks_per_rev) * float(self.params.gear_ratio)
        return 2.0 * pi * (float(dN) / denom)

    def update_from_ticks(self, dN_L: int, dN_R: int) -> Pose2D:
        """
        Update pose from incremental ticks (left, right). Returns updated pose.
        """
        dphi_L = self.ticks_to_wheel_angle(dN_L)
        dphi_R = self.ticks_to_wheel_angle(dN_R)

        ds_L = self.params.r_l * dphi_L
        ds_R = self.params.r_r * dphi_R

        delta_s = 0.5 * (ds_R + ds_L)
        delta_theta = (ds_R - ds_L) / self.params.b

        x, y, th = self.pose.x, self.pose.y, self.pose.theta

        # Exact integration for constant v, omega over the sample:
        # If delta_theta is tiny, fall back to straight-line to avoid 0/0.
        eps = 1e-12
        if abs(delta_theta) < eps:
            x += delta_s * cos(th)
            y += delta_s * sin(th)
            th = normalize_angle(th + delta_theta)
        else:
            R = delta_s / delta_theta  # signed radius
            x += R * (sin(th + delta_theta) - sin(th))
            y += -R * (cos(th + delta_theta) - cos(th))
            th = normalize_angle(th + delta_theta)

        self.pose = Pose2D(x, y, th)
        return self.pose


def _demo() -> None:
    """
    Minimal demo: drive an arc for 200 steps and print final pose.
    """
    params = DiffDriveParams(
        r_l=0.05, r_r=0.05, b=0.30, ticks_per_rev=2048, gear_ratio=1.0
    )
    odo = DifferentialDriveOdometry(params)

    # Simulate: right wheel advances more than left wheel -> turning left
    for _ in range(200):
        odo.update_from_ticks(dN_L=40, dN_R=60)

    print("Final pose:", odo.pose)


if __name__ == "__main__":
    _demo()
