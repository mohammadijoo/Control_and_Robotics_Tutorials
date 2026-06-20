# Chapter11_Lesson1.py
"""
Autonomous Mobile Robots (Control Engineering)
Chapter 11: SLAM I — Filter-Based SLAM
Lesson 1: The SLAM Problem Formulation

This script implements a *minimal* filter-based SLAM core using an EKF over the
augmented state y = [x, y, theta, m1x, m1y, ..., mNx, mNy].

Pedagogical choices (for Lesson 1):
- Known data association (each measurement is already matched to a landmark id).
- Simple differential-drive motion model (unicycle).
- Range-bearing landmark observations.
- Gaussian noise; standard EKF predict-update.

Note: EKF-SLAM structure/limitations are treated in Lesson 2; here we focus on
the probabilistic formulation and the recursion implemented by a filter.
"""

from __future__ import annotations
import numpy as np


def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def motion_model(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    """
    Unicycle motion model.
    State: x = [px, py, theta]
    Control: u = [v, omega, dt]
    """
    px, py, th = x
    v, w, dt = u
    if abs(w) < 1e-12:
        px2 = px + v * dt * np.cos(th)
        py2 = py + v * dt * np.sin(th)
        th2 = th
    else:
        px2 = px + (v / w) * (np.sin(th + w * dt) - np.sin(th))
        py2 = py - (v / w) * (np.cos(th + w * dt) - np.cos(th))
        th2 = th + w * dt
    return np.array([px2, py2, wrap_angle(th2)], dtype=float)


def jacobian_motion_wrt_state(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    """
    Jacobian Fx = d f(x,u) / d x  (3x3).
    """
    px, py, th = x
    v, w, dt = u
    Fx = np.eye(3, dtype=float)

    if abs(w) < 1e-12:
        Fx[0, 2] = -v * dt * np.sin(th)
        Fx[1, 2] =  v * dt * np.cos(th)
    else:
        th2 = th + w * dt
        Fx[0, 2] = (v / w) * (np.cos(th2) - np.cos(th))
        Fx[1, 2] = (v / w) * (np.sin(th2) - np.sin(th))
    return Fx


def measurement_model_pose_landmark(x: np.ndarray, mi: np.ndarray) -> np.ndarray:
    """
    Range-bearing measurement to landmark i.
    z = [range, bearing], bearing is relative to robot heading.
    """
    px, py, th = x
    mx, my = mi
    dx = mx - px
    dy = my - py
    r = np.sqrt(dx * dx + dy * dy)
    b = np.arctan2(dy, dx) - th
    return np.array([r, wrap_angle(b)], dtype=float)


def jacobian_measurement_wrt_pose_and_landmark(x: np.ndarray, mi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Returns (H_x, H_m) where:
    H_x = d h / d x  (2x3)
    H_m = d h / d m_i (2x2)
    """
    px, py, th = x
    mx, my = mi
    dx = mx - px
    dy = my - py
    q = dx * dx + dy * dy
    r = np.sqrt(q)

    # Robustness against division by zero
    if q < 1e-12:
        q = 1e-12
        r = np.sqrt(q)

    # dr/dx = -dx/r, dr/dy = -dy/r, dr/dtheta = 0
    # dphi/dx =  dy/q, dphi/dy = -dx/q, dphi/dtheta = -1
    Hx = np.array([
        [-dx / r, -dy / r, 0.0],
        [ dy / q, -dx / q, -1.0]
    ], dtype=float)

    # dr/dmx = dx/r, dr/dmy = dy/r
    # dphi/dmx = -dy/q, dphi/dmy = dx/q
    Hm = np.array([
        [ dx / r,  dy / r],
        [-dy / q,  dx / q]
    ], dtype=float)
    return Hx, Hm


def ekf_slam_predict(y: np.ndarray, P: np.ndarray, u: np.ndarray, Q_pose: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    EKF predict step on augmented state.
    Q_pose: 3x3 process noise covariance on pose (in state coordinates).
    """
    n = y.size
    assert P.shape == (n, n)
    assert Q_pose.shape == (3, 3)

    x = y[:3].copy()
    m = y[3:].copy()

    Fx = jacobian_motion_wrt_state(x, u)
    x_pred = motion_model(x, u)

    # Augmented transition Jacobian
    F = np.eye(n, dtype=float)
    F[:3, :3] = Fx

    # Augmented process noise (pose only)
    Q = np.zeros((n, n), dtype=float)
    Q[:3, :3] = Q_pose

    y_pred = np.concatenate([x_pred, m])
    P_pred = F @ P @ F.T + Q
    return y_pred, P_pred


def ekf_slam_update_landmark(
    y: np.ndarray,
    P: np.ndarray,
    z: np.ndarray,
    landmark_id: int,
    R_meas: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    EKF update for a single landmark observation with known data association.

    landmark_id: 0-based index into landmark list.
    z: [range, bearing]
    R_meas: 2x2 measurement noise covariance
    Returns: (y_upd, P_upd, innovation)
    """
    n = y.size
    assert P.shape == (n, n)
    assert z.shape == (2,)
    assert R_meas.shape == (2, 2)

    x = y[:3]
    i = landmark_id
    idx = 3 + 2 * i
    mi = y[idx:idx + 2]

    z_hat = measurement_model_pose_landmark(x, mi)
    innov = np.array([z[0] - z_hat[0], wrap_angle(z[1] - z_hat[1])], dtype=float)

    Hx, Hm = jacobian_measurement_wrt_pose_and_landmark(x, mi)

    H = np.zeros((2, n), dtype=float)
    H[:, :3] = Hx
    H[:, idx:idx + 2] = Hm

    S = H @ P @ H.T + R_meas
    K = P @ H.T @ np.linalg.inv(S)

    y_upd = y + K @ innov
    y_upd[2] = wrap_angle(y_upd[2])

    # Joseph form for covariance update (numerically safer)
    I = np.eye(n, dtype=float)
    P_upd = (I - K @ H) @ P @ (I - K @ H).T + K @ R_meas @ K.T

    return y_upd, P_upd, innov


def demo():
    np.random.seed(7)

    # --- Ground truth landmarks (unknown to the filter) ---
    landmarks = np.array([
        [4.0,  2.0],
        [8.0, -1.0],
        [2.0, -3.0],
    ], dtype=float)
    N = landmarks.shape[0]

    # --- True state ---
    x_true = np.array([0.0, 0.0, 0.0], dtype=float)

    # --- EKF-SLAM initial belief ---
    y = np.zeros(3 + 2 * N, dtype=float)
    y[:3] = np.array([0.0, 0.0, 0.0], dtype=float)

    # Initialize landmark guesses (poor prior)
    y[3:] = np.array([3.0,  1.0,  7.0,  0.0,  1.0, -2.0], dtype=float)

    P = np.eye(y.size, dtype=float) * 1e-3
    # Large uncertainty on map components
    for i in range(N):
        idx = 3 + 2 * i
        P[idx:idx + 2, idx:idx + 2] = np.eye(2) * 4.0

    # Noise models
    Q_pose = np.diag([0.02**2, 0.02**2, (np.deg2rad(1.0))**2])
    R_meas = np.diag([0.10**2, (np.deg2rad(2.0))**2])

    # Controls: (v, omega, dt)
    U = [
        np.array([1.0,  0.10, 1.0]),
        np.array([1.0,  0.00, 1.0]),
        np.array([1.0, -0.10, 1.0]),
        np.array([1.0,  0.00, 1.0]),
    ]

    for t, u in enumerate(U, start=1):
        # True motion with small perturbation
        x_true = motion_model(x_true, u) + np.random.multivariate_normal(np.zeros(3), Q_pose)
        x_true[2] = wrap_angle(x_true[2])

        # Predict
        y, P = ekf_slam_predict(y, P, u, Q_pose)

        # Observe all landmarks (known association), generate noisy measurements
        for i in range(N):
            z_true = measurement_model_pose_landmark(x_true, landmarks[i])
            z_noisy = z_true + np.random.multivariate_normal(np.zeros(2), R_meas)
            z_noisy[1] = wrap_angle(z_noisy[1])

            y, P, innov = ekf_slam_update_landmark(y, P, z_noisy, i, R_meas)

        print(f"t={t}: pose mean = {y[:3].round(3)}")

    print("\nFinal pose estimate:", y[:3])
    print("Final landmark estimates:")
    for i in range(N):
        idx = 3 + 2 * i
        print(f"  m{i}: {y[idx:idx+2]}")
    print("\nSmallest eigenvalue of P:", np.linalg.eigvalsh(P).min())


if __name__ == "__main__":
    demo()
