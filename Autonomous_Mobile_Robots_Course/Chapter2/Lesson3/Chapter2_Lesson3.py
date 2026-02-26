"""
Chapter2_Lesson3.py
Autonomous Mobile Robots — Chapter 2, Lesson 3
Car-Like / Ackermann Steering Kinematics

This script implements:
1) Ackermann steering geometry (left/right wheel steering angles).
2) Bicycle (single-track) kinematic model.
3) Exact discrete-time integration for constant (v, delta) over dt.
4) A small simulation demo, plus an optional plot.

Recommended Python libs for AMR kinematics:
- numpy (arrays, numerics)
- matplotlib (plotting)
- scipy (optional for integration/optimization)
- sympy (optional for symbolic checks)
- ROS 1/2 interfaces (optional): geometry_msgs, nav_msgs, ackermann_msgs

Install (if needed):
    pip install numpy matplotlib
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass
class Pose2D:
    """Planar pose in SE(2) coordinates (x, y, yaw)."""
    x: float
    y: float
    theta: float  # yaw [rad]


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a


def curvature_from_steering(delta: float, L: float) -> float:
    """
    Bicycle model curvature kappa = tan(delta)/L.
    delta: virtual front wheel steering angle [rad]
    L: wheelbase [m]
    """
    return math.tan(delta) / L


def ackermann_wheel_angles_from_virtual(delta: float, L: float, W: float) -> Tuple[float, float]:
    """
    Convert a *virtual* front steering angle delta (bicycle model) to the left/right
    front wheel steering angles (Ackermann condition) for a vehicle with wheelbase L
    and track width W.

    Convention:
      - delta > 0 means turning left (CCW yaw).
      - Returned: (delta_left, delta_right) in radians.

    Derivation:
      kappa = tan(delta)/L, so turning radius R = 1/kappa (if kappa != 0).
      Then:
        delta_left  = atan(L / (R - W/2))
        delta_right = atan(L / (R + W/2))
      for a left turn (R > 0). For right turn, signs are handled by R.
    """
    kappa = curvature_from_steering(delta, L)
    if abs(kappa) < 1e-12:
        return 0.0, 0.0

    R = 1.0 / kappa  # signed
    # Denominators are signed; atan handles sign. Avoid division by zero.
    dl = math.atan2(L, (R - W / 2.0))
    dr = math.atan2(L, (R + W / 2.0))
    return dl, dr


def step_bicycle_euler(p: Pose2D, v: float, delta: float, L: float, dt: float) -> Pose2D:
    """
    Forward Euler discretization of the bicycle model:
        x_dot = v cos(theta)
        y_dot = v sin(theta)
        theta_dot = v/L * tan(delta)
    """
    omega = v * math.tan(delta) / L
    x_new = p.x + dt * v * math.cos(p.theta)
    y_new = p.y + dt * v * math.sin(p.theta)
    th_new = wrap_to_pi(p.theta + dt * omega)
    return Pose2D(x_new, y_new, th_new)


def step_bicycle_exact(p: Pose2D, v: float, delta: float, L: float, dt: float) -> Pose2D:
    """
    Exact integration of the bicycle model over dt assuming v and delta are constant.
    Let omega = v/L * tan(delta). Then:

    If |omega| is small:
        x_{k+1} = x_k + v dt cos(theta_k)
        y_{k+1} = y_k + v dt sin(theta_k)
        theta_{k+1} = theta_k + omega dt
    Else:
        x_{k+1} = x_k + (v/omega) [ sin(theta_k + omega dt) - sin(theta_k) ]
        y_{k+1} = y_k + (v/omega) [ -cos(theta_k + omega dt) + cos(theta_k) ]
        theta_{k+1} = theta_k + omega dt
    """
    omega = v * math.tan(delta) / L
    if abs(omega) < 1e-10:
        return step_bicycle_euler(p, v, delta, L, dt)

    th2 = p.theta + omega * dt
    x_new = p.x + (v / omega) * (math.sin(th2) - math.sin(p.theta))
    y_new = p.y + (v / omega) * (-math.cos(th2) + math.cos(p.theta))
    th_new = wrap_to_pi(th2)
    return Pose2D(x_new, y_new, th_new)


def simulate_piecewise_constant(
    p0: Pose2D,
    controls: List[Tuple[float, float, float]],
    L: float,
    method: str = "exact",
) -> List[Pose2D]:
    """
    Simulate with piecewise-constant controls.
    controls: list of (v, delta, dt).
    method: "exact" or "euler"
    Returns: list of poses including initial.
    """
    poses = [p0]
    p = p0
    for v, delta, dt in controls:
        if method == "exact":
            p = step_bicycle_exact(p, v, delta, L, dt)
        elif method == "euler":
            p = step_bicycle_euler(p, v, delta, L, dt)
        else:
            raise ValueError("method must be 'exact' or 'euler'")
        poses.append(p)
    return poses


def demo():
    # Vehicle parameters
    L = 2.7   # wheelbase [m]
    W = 1.6   # track width [m]

    # Initial pose
    p0 = Pose2D(0.0, 0.0, 0.0)

    # Control sequence: (v, delta, dt)
    # Drive forward with a constant steering angle, then straighten.
    controls = [
        (2.0, math.radians(15.0), 1.0),
        (2.0, math.radians(15.0), 1.0),
        (2.0, math.radians(0.0),  1.0),
        (2.0, math.radians(0.0),  1.0),
    ]

    poses_exact = simulate_piecewise_constant(p0, controls, L=L, method="exact")
    poses_euler = simulate_piecewise_constant(p0, controls, L=L, method="euler")

    pe = poses_exact[-1]
    pu = poses_euler[-1]
    print("Final pose (exact):", pe)
    print("Final pose (euler):", pu)

    # Show Ackermann left/right wheel angles corresponding to the *virtual* delta
    delta_virtual = math.radians(15.0)
    dl, dr = ackermann_wheel_angles_from_virtual(delta_virtual, L=L, W=W)
    print(f"Virtual steering delta = {math.degrees(delta_virtual):.2f} deg")
    print(f"Ackermann delta_left  = {math.degrees(dl):.2f} deg")
    print(f"Ackermann delta_right = {math.degrees(dr):.2f} deg")

    # Optional plot
    try:
        import matplotlib.pyplot as plt
        xe = [p.x for p in poses_exact]
        ye = [p.y for p in poses_exact]
        xu = [p.x for p in poses_euler]
        yu = [p.y for p in poses_euler]
        plt.figure()
        plt.plot(xe, ye, marker="o", label="exact")
        plt.plot(xu, yu, marker="x", label="euler")
        plt.axis("equal")
        plt.grid(True)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("Bicycle model simulation (exact vs euler)")
        plt.legend()
        plt.show()
    except Exception as e:
        print("Plot skipped:", e)


if __name__ == "__main__":
    demo()
