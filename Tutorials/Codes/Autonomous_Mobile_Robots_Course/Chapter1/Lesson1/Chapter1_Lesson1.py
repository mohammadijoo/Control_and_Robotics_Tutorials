"""Chapter1_Lesson1.py
Autonomous Mobile Robots (Control Engineering) — Chapter 1, Lesson 1
Lesson: What Makes Mobility Different from Manipulation

This script provides a minimal, self-contained numerical experiment that contrasts:
  (i) a planar mobile base (differential-drive kinematics) and
  (ii) a 2R planar manipulator (holonomic joint-space actuation).

It generates CSV files for both trajectories and (optionally) plots them.

Suggested robotics libraries to explore next (not required for this script):
  - roboticstoolbox-python (Peter Corke) for manipulator models and Jacobians
  - spatialmath-python for SE(2)/SE(3) utilities
  - ROS 2 (rclpy) for integration with real robots

Tested with: Python 3.10+
"""

from __future__ import annotations

import math
import csv
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_PLOT = True
except Exception:
    HAS_PLOT = False


@dataclass
class Pose2:
    x: float
    y: float
    theta: float  # yaw


def integrate_diff_drive(p0: Pose2, v: float, w: float, dt: float, steps: int) -> List[Pose2]:
    """Forward-Euler integration of the standard planar kinematic model:
        x_dot = v cos(theta), y_dot = v sin(theta), theta_dot = w
    """
    traj = [p0]
    p = Pose2(p0.x, p0.y, p0.theta)
    for _ in range(steps):
        p.x += dt * v * math.cos(p.theta)
        p.y += dt * v * math.sin(p.theta)
        p.theta += dt * w
        traj.append(Pose2(p.x, p.y, p.theta))
    return traj


def fk_2r(q1: float, q2: float, l1: float = 1.0, l2: float = 0.7) -> Tuple[float, float]:
    """Planar 2R forward kinematics (end-effector position)."""
    x = l1 * math.cos(q1) + l2 * math.cos(q1 + q2)
    y = l1 * math.sin(q1) + l2 * math.sin(q1 + q2)
    return x, y


def jacobian_2r(q1: float, q2: float, l1: float = 1.0, l2: float = 0.7) -> np.ndarray:
    """Planar 2R Jacobian for end-effector linear velocity: [x_dot; y_dot] = J(q) [q1_dot; q2_dot]."""
    s1 = math.sin(q1)
    c1 = math.cos(q1)
    s12 = math.sin(q1 + q2)
    c12 = math.cos(q1 + q2)

    j11 = -l1 * s1 - l2 * s12
    j12 = -l2 * s12
    j21 =  l1 * c1 + l2 * c12
    j22 =  l2 * c12
    return np.array([[j11, j12], [j21, j22]], dtype=float)


def simulate_2r(q10: float, q20: float, qdot1: float, qdot2: float, dt: float, steps: int) -> Tuple[np.ndarray, np.ndarray]:
    """Integrate joint angles and compute end-effector trajectory."""
    q1, q2 = q10, q20
    qs = np.zeros((steps + 1, 2), dtype=float)
    xs = np.zeros((steps + 1, 2), dtype=float)

    for k in range(steps + 1):
        qs[k, :] = [q1, q2]
        xs[k, :] = fk_2r(q1, q2)
        q1 += dt * qdot1
        q2 += dt * qdot2

    return qs, xs


def write_mobile_csv(path: str, traj: List[Pose2], dt: float) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "theta"])
        for k, p in enumerate(traj):
            w.writerow([k * dt, p.x, p.y, p.theta])


def write_manip_csv(path: str, qs: np.ndarray, xs: np.ndarray, dt: float) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "q1", "q2", "xe", "ye"])
        for k in range(qs.shape[0]):
            w.writerow([k * dt, qs[k, 0], qs[k, 1], xs[k, 0], xs[k, 1]])


def main() -> None:
    dt = 0.02
    T = 8.0
    steps = int(T / dt)

    # Mobile base inputs (body-frame): v (m/s), w (rad/s)
    v = 0.5
    w = 0.35
    mobile_traj = integrate_diff_drive(Pose2(0.0, 0.0, 0.0), v=v, w=w, dt=dt, steps=steps)
    write_mobile_csv("Chapter1_Lesson1_mobile.csv", mobile_traj, dt)

    # Manipulator joint-rate inputs: qdot (rad/s)
    qdot1, qdot2 = 0.25, -0.15
    qs, xs = simulate_2r(q10=0.2, q20=0.9, qdot1=qdot1, qdot2=qdot2, dt=dt, steps=steps)
    write_manip_csv("Chapter1_Lesson1_manipulator.csv", qs, xs, dt)

    # Demonstrate instantaneous vs integrable constraints:
    # - Mobile base cannot have arbitrary instantaneous (x_dot, y_dot) at fixed theta when lateral velocity is constrained.
    # - Manipulator can set any (q1_dot, q2_dot) instantaneously (actuation in joint space is holonomic).
    J = jacobian_2r(q1=qs[0, 0], q2=qs[0, 1])
    v_ee = J @ np.array([qdot1, qdot2])
    print("2R end-effector velocity at t=0:", v_ee)

    if HAS_PLOT:
        mx = [p.x for p in mobile_traj]
        my = [p.y for p in mobile_traj]

        plt.figure()
        plt.plot(mx, my)
        plt.axis("equal")
        plt.title("Mobile base trajectory (x-y)")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")

        plt.figure()
        plt.plot(xs[:, 0], xs[:, 1])
        plt.axis("equal")
        plt.title("2R manipulator end-effector trajectory (x-y)")
        plt.xlabel("x_e [m]")
        plt.ylabel("y_e [m]")

        plt.show()

    print("Wrote: Chapter1_Lesson1_mobile.csv and Chapter1_Lesson1_manipulator.csv")


if __name__ == "__main__":
    main()
