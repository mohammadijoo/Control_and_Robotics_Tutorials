"""
Chapter 5 — Odometry and Dead Reckoning
Lesson 2: IMU Integration for Ground Robots

Filename: Chapter5_Lesson2.py

This script demonstrates deterministic strapdown IMU integration for a ground robot:
- Gyro integration for attitude (quaternion)
- Accel integration for velocity/position (in navigation frame)
- Optional planar constraints (z=0, roll=pitch=0 projection)
- Optional complementary fusion of yaw with a wheel-odometry yaw estimate

Dependencies: numpy (standard). matplotlib is optional for plotting.
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Tuple, Optional

import numpy as np


# -----------------------------
# Quaternion utilities (w, x, y, z)
# -----------------------------
def q_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n <= 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def q_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=float)


def q_from_delta_theta(dtheta: np.ndarray) -> np.ndarray:
    """
    Convert rotation vector (rad) to quaternion using exponential map.
    dtheta: shape (3,)
    """
    angle = float(np.linalg.norm(dtheta))
    if angle < 1e-12:
        # Small-angle: sin(a/2) ~ a/2
        half = 0.5
        return q_normalize(np.array([1.0, half*dtheta[0], half*dtheta[1], half*dtheta[2]], dtype=float))
    axis = dtheta / angle
    half = 0.5 * angle
    s = math.sin(half)
    return np.array([math.cos(half), axis[0]*s, axis[1]*s, axis[2]*s], dtype=float)


def q_to_R(q: np.ndarray) -> np.ndarray:
    """Quaternion to rotation matrix R_NB that maps body vectors to nav frame."""
    w, x, y, z = q
    # Standard quaternion rotation
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ], dtype=float)


def yaw_from_q(q: np.ndarray) -> float:
    """Return yaw (rad) for Z-up navigation frame using ZYX convention."""
    w, x, y, z = q
    # yaw = atan2(2(w z + x y), 1 - 2(y^2 + z^2))
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))


def q_from_yaw(yaw: float) -> np.ndarray:
    """Quaternion for pure yaw about nav z-axis."""
    half = 0.5 * yaw
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)


def wrap_pi(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (a + math.pi) % (2*math.pi) - math.pi
    return a


@dataclass
class IMUParams:
    g: float = 9.80665
    b_g: np.ndarray = np.zeros(3)  # rad/s bias
    b_a: np.ndarray = np.zeros(3)  # m/s^2 bias
    enforce_planar: bool = True
    fuse_yaw_with_wheel: bool = True
    tau_yaw: float = 2.0  # s, complementary time constant for yaw fusion


@dataclass
class State:
    p: np.ndarray  # position in nav frame (m), shape (3,)
    v: np.ndarray  # velocity in nav frame (m/s), shape (3,)
    q: np.ndarray  # quaternion (w,x,y,z), body->nav


def strapdown_integrate(
    t: np.ndarray,
    gyro: np.ndarray,
    accel: np.ndarray,
    params: IMUParams,
    wheel_yaw: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Integrate IMU to produce position, velocity, yaw over time.
    Inputs:
      t: (N,) seconds, strictly increasing
      gyro: (N,3) rad/s body rates (wx, wy, wz)
      accel: (N,3) m/s^2 specific force (ax, ay, az) in body frame
      wheel_yaw: (N,) rad, optional yaw estimate from wheel odometry
    Returns:
      p_hist: (N,3), v_hist: (N,3), yaw_hist: (N,)
    """
    N = t.shape[0]
    p = np.zeros(3, dtype=float)
    v = np.zeros(3, dtype=float)
    q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # identity

    p_hist = np.zeros((N,3), dtype=float)
    v_hist = np.zeros((N,3), dtype=float)
    yaw_hist = np.zeros(N, dtype=float)

    g_nav = np.array([0.0, 0.0, -params.g], dtype=float)  # Z-up nav; gravity down

    for k in range(N):
        if k == 0:
            dt = float(t[1] - t[0])
        else:
            dt = float(t[k] - t[k-1])

        # 1) Attitude propagation
        omega = gyro[k] - params.b_g
        dq = q_from_delta_theta(omega * dt)
        q = q_normalize(q_mul(q, dq))

        # Optional: enforce planar roll/pitch constraint by projecting to pure yaw
        if params.enforce_planar:
            yaw = yaw_from_q(q)
            q = q_from_yaw(yaw)

        # 2) Acceleration to navigation frame
        f_b = accel[k] - params.b_a  # specific force (body)
        R = q_to_R(q)                # body->nav
        a_nav = R @ f_b + g_nav      # a = R f + g

        # 3) Velocity/position integration (semi-implicit Euler)
        v = v + a_nav * dt
        p = p + v * dt + 0.5 * a_nav * (dt**2)

        # Enforce ground constraint in z (common for wheeled robots on flat terrain)
        if params.enforce_planar:
            p[2] = 0.0
            v[2] = 0.0

        # 4) Optional yaw complementary fusion with wheel yaw
        yaw = yaw_from_q(q)
        if params.fuse_yaw_with_wheel and wheel_yaw is not None:
            # Continuous-time complementary: dpsi = (psi_w - psi_hat)/tau
            # Discrete form:
            alpha = math.exp(-dt / max(params.tau_yaw, 1e-6))  # high-pass on gyro yaw
            yaw = wrap_pi(alpha * yaw + (1.0 - alpha) * float(wheel_yaw[k]))
            q = q_from_yaw(yaw)

        p_hist[k] = p
        v_hist[k] = v
        yaw_hist[k] = yaw

    return p_hist, v_hist, yaw_hist


# -----------------------------
# Demo: simulate a ground robot with a constant-speed turn
# -----------------------------
def simulate_ground_robot(N: int = 2000, dt: float = 0.01) -> None:
    """
    Simulate:
      - Robot drives at constant speed v0 and constant yaw rate w0
      - IMU measures gyro and specific force with bias + noise
      - Wheel odom provides yaw with lower drift (for demonstration)
      - Integrate IMU with/without yaw fusion and print final errors
    """
    t = np.arange(N) * dt

    # Ground truth motion (planar)
    v0 = 1.2         # m/s
    w0 = 0.20        # rad/s
    yaw_true = w0 * t
    x_true = (v0 / w0) * np.sin(yaw_true)
    y_true = (v0 / w0) * (1.0 - np.cos(yaw_true))

    # True acceleration in nav frame (centripetal for constant turn)
    ax_nav = v0 * w0 * np.cos(yaw_true)   # derivative of v_x
    ay_nav = v0 * w0 * np.sin(yaw_true)   # derivative of v_y
    a_nav = np.stack([ax_nav, ay_nav, np.zeros_like(t)], axis=1)

    # Rotation body->nav is pure yaw
    def Rz(yaw: np.ndarray) -> np.ndarray:
        c = np.cos(yaw); s = np.sin(yaw)
        R = np.zeros((yaw.size, 3, 3))
        R[:,0,0] = c;  R[:,0,1] = -s
        R[:,1,0] = s;  R[:,1,1] =  c
        R[:,2,2] = 1.0
        return R

    R_nb = Rz(yaw_true)                 # body->nav
    g_nav = np.array([0.0, 0.0, -9.80665])
    # specific force in body: f_b = R_bn (a_nav - g_nav)
    f_b = np.einsum('nij,nj->ni', np.transpose(R_nb, (0,2,1)), (a_nav - g_nav))

    # Gyro measures body rates (only wz) in body frame
    gyro_true = np.zeros((N,3))
    gyro_true[:,2] = w0

    rng = np.random.default_rng(1)
    b_g = np.array([0.0, 0.0, 0.01])      # rad/s bias (yaw gyro bias)
    b_a = np.array([0.05, -0.02, 0.00])   # m/s^2 bias (horizontal accel bias)

    gyro_meas = gyro_true + b_g + rng.normal(0.0, 0.002, size=(N,3))
    accel_meas = f_b + b_a + rng.normal(0.0, 0.05, size=(N,3))

    # Wheel yaw (drifts less than gyro for this demo)
    wheel_yaw = yaw_true + rng.normal(0.0, 0.01, size=(N,))
    # add a slow drift
    wheel_yaw = wheel_yaw + 0.002 * t

    # Integrate IMU only
    p1, v1, yaw1 = strapdown_integrate(
        t, gyro_meas, accel_meas,
        IMUParams(b_g=b_g*0.0, b_a=b_a*0.0, enforce_planar=True, fuse_yaw_with_wheel=False),
        wheel_yaw=None
    )

    # Integrate IMU + yaw fusion with wheel
    p2, v2, yaw2 = strapdown_integrate(
        t, gyro_meas, accel_meas,
        IMUParams(b_g=b_g*0.0, b_a=b_a*0.0, enforce_planar=True, fuse_yaw_with_wheel=True, tau_yaw=2.0),
        wheel_yaw=wheel_yaw
    )

    p_true = np.stack([x_true, y_true, np.zeros_like(t)], axis=1)

    e1 = p1[-1,:2] - p_true[-1,:2]
    e2 = p2[-1,:2] - p_true[-1,:2]

    print("Final position error (IMU only) [m]:", e1)
    print("Final position error (IMU + yaw fusion) [m]:", e2)
    print("Final yaw error (IMU only) [deg]:", np.degrees(wrap_pi(yaw1[-1] - yaw_true[-1])))
    print("Final yaw error (fused) [deg]:", np.degrees(wrap_pi(yaw2[-1] - yaw_true[-1])))

    # Optional plot
    try:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(p_true[:,0], p_true[:,1], label="true")
        plt.plot(p1[:,0], p1[:,1], label="imu-only")
        plt.plot(p2[:,0], p2[:,1], label="imu+yaw fusion")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.axis("equal")
        plt.legend()
        plt.title("Chapter5 Lesson2: IMU dead reckoning drift (demo)")
        plt.show()
    except Exception:
        pass


if __name__ == "__main__":
    simulate_ground_robot()
