"""Chapter 2 - Lesson 2: Differential Drive Kinematics

University-level reference implementation of:
  1) Forward kinematics: wheel rates -> body twist -> pose update
  2) Inverse kinematics: desired (v, w) -> wheel rates
  3) Exact discrete-time integration under piecewise-constant (v, w)

Assumptions:
  - Planar motion (x, y, theta)
  - Pure rolling, no lateral slip
  - Wheel radius r and axle length L are known constants
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class DiffDriveParams:
    r: float  # wheel radius [m]
    L: float  # axle length / wheel separation [m]


def body_twist_from_wheels(phi_dot_l: float, phi_dot_r: float, p: DiffDriveParams) -> tuple[float, float]:
    """Map wheel angular rates (rad/s) to body twist (v [m/s], w [rad/s])."""
    v_l = p.r * phi_dot_l
    v_r = p.r * phi_dot_r
    v = 0.5 * (v_r + v_l)
    w = (v_r - v_l) / p.L
    return float(v), float(w)


def wheels_from_body_twist(v: float, w: float, p: DiffDriveParams) -> tuple[float, float]:
    """Inverse mapping: desired (v,w) to wheel angular rates (phi_dot_l, phi_dot_r)."""
    phi_dot_r = (v + 0.5 * p.L * w) / p.r
    phi_dot_l = (v - 0.5 * p.L * w) / p.r
    return float(phi_dot_l), float(phi_dot_r)


def integrate_pose_exact(pose: np.ndarray, v: float, w: float, dt: float) -> np.ndarray:
    """Exact integration over dt for constant (v,w)."""
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    if abs(w) < 1e-12:
        return np.array([x + v * dt * np.cos(th),
                         y + v * dt * np.sin(th),
                         th], dtype=float)

    th2 = th + w * dt
    x2 = x + (v / w) * (np.sin(th2) - np.sin(th))
    y2 = y - (v / w) * (np.cos(th2) - np.cos(th))
    return np.array([x2, y2, th2], dtype=float)


def step_from_wheels(pose: np.ndarray, phi_dot_l: float, phi_dot_r: float, p: DiffDriveParams, dt: float) -> np.ndarray:
    v, w = body_twist_from_wheels(phi_dot_l, phi_dot_r, p)
    return integrate_pose_exact(pose, v, w, dt)


def update_from_encoder_increments(pose: np.ndarray, dphi_l: float, dphi_r: float, p: DiffDriveParams) -> np.ndarray:
    """Pose update from wheel encoder increments over a sampling interval.

    Uses the same exact integration idea with:
      ds_l = r * dphi_l, ds_r = r * dphi_r
      ds   = (ds_r + ds_l)/2
      dth  = (ds_r - ds_l)/L
    """
    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    ds_l = p.r * dphi_l
    ds_r = p.r * dphi_r
    ds = 0.5 * (ds_r + ds_l)
    dth = (ds_r - ds_l) / p.L

    if abs(dth) < 1e-12:
        return np.array([x + ds * np.cos(th),
                         y + ds * np.sin(th),
                         th], dtype=float)

    th2 = th + dth
    # v/w = ds/dth for incremental form
    x2 = x + (ds / dth) * (np.sin(th2) - np.sin(th))
    y2 = y - (ds / dth) * (np.cos(th2) - np.cos(th))
    return np.array([x2, y2, th2], dtype=float)


def simulate_constant_wheels(pose0: np.ndarray, phi_dot_l: float, phi_dot_r: float,
                             p: DiffDriveParams, dt: float, N: int) -> np.ndarray:
    pose = np.array(pose0, dtype=float)
    traj = np.zeros((N + 1, 3), dtype=float)
    traj[0] = pose
    for k in range(N):
        pose = step_from_wheels(pose, phi_dot_l, phi_dot_r, p, dt)
        traj[k + 1] = pose
    return traj


if __name__ == "__main__":
    p = DiffDriveParams(r=0.05, L=0.30)
    pose0 = np.array([0.0, 0.0, 0.0], dtype=float)

    # Example: left wheel slower than right -> turn left
    phi_dot_l, phi_dot_r = 5.0, 8.0  # rad/s
    dt, N = 0.1, 50
    traj = simulate_constant_wheels(pose0, phi_dot_l, phi_dot_r, p, dt, N)

    v, w = body_twist_from_wheels(phi_dot_l, phi_dot_r, p)
    print(f"v = {v:.4f} m/s, w = {w:.4f} rad/s")
    print("Final pose [x, y, theta] =", traj[-1])

    # Inverse check
    pl, pr = wheels_from_body_twist(v, w, p)
    print("Recovered wheel rates [phi_dot_l, phi_dot_r] =", (pl, pr))

    # Encoder-increment example: integrate the same motion using increments
    dphi_l = phi_dot_l * dt
    dphi_r = phi_dot_r * dt
    pose = pose0.copy()
    for _ in range(N):
        pose = update_from_encoder_increments(pose, dphi_l, dphi_r, p)
    print("Final pose from encoder increments =", pose)
