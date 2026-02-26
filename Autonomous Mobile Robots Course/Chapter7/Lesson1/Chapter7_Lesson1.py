# Chapter7_Lesson1.py
"""
Chapter 7 - Lesson 1: EKF Localization Pipeline (mobile framing)

A minimal, from-scratch EKF localization demo:
- State: x = [x, y, theta]^T in world frame W
- Control: u = [v, omega]^T measured in body frame B
- Measurement: range-bearing to known landmarks (2D)

Dependencies: numpy, matplotlib (optional for plotting)
"""

from __future__ import annotations
import math
import numpy as np

def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def motion_model(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """
    x_{k+1} = f(x_k, u_k) (deterministic part)
    x = [x, y, theta], u = [v, omega]
    """
    px, py, th = float(x[0]), float(x[1]), float(x[2])
    v, w = float(u[0]), float(u[1])
    nx = np.zeros((3, 1), dtype=float)
    nx[0, 0] = px + dt * v * math.cos(th)
    nx[1, 0] = py + dt * v * math.sin(th)
    nx[2, 0] = wrap_angle(th + dt * w)
    return nx

def jacobian_F(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """F_k = d f / d x evaluated at (x,u)."""
    th = float(x[2])
    v = float(u[0])
    F = np.eye(3, dtype=float)
    F[0, 2] = -dt * v * math.sin(th)
    F[1, 2] =  dt * v * math.cos(th)
    return F

def jacobian_L(x: np.ndarray, dt: float) -> np.ndarray:
    """
    L_k = d f / d w where process noise w = [n_v, n_omega]^T
    added to control: v_hat = v + n_v, omega_hat = omega + n_omega
    """
    th = float(x[2])
    L = np.zeros((3, 2), dtype=float)
    L[0, 0] = dt * math.cos(th)
    L[1, 0] = dt * math.sin(th)
    L[2, 1] = dt
    return L

def meas_model_range_bearing(x: np.ndarray, lm: np.ndarray) -> np.ndarray:
    """
    z = h(x) = [range, bearing]^T to a landmark lm=[mx,my]^T in world frame.
    bearing is relative to robot heading theta.
    """
    px, py, th = float(x[0]), float(x[1]), float(x[2])
    mx, my = float(lm[0]), float(lm[1])
    dx = mx - px
    dy = my - py
    r = math.sqrt(dx*dx + dy*dy)
    b = wrap_angle(math.atan2(dy, dx) - th)
    return np.array([[r], [b]], dtype=float)

def jacobian_H_range_bearing(x: np.ndarray, lm: np.ndarray) -> np.ndarray:
    """H = d h / d x for range-bearing."""
    px, py, th = float(x[0]), float(x[1]), float(x[2])
    mx, my = float(lm[0]), float(lm[1])
    dx = mx - px
    dy = my - py
    q = dx*dx + dy*dy
    r = math.sqrt(q)
    # Guard against division by zero
    eps = 1e-12
    q = max(q, eps)
    r = max(r, eps)

    H = np.zeros((2, 3), dtype=float)
    # range derivatives
    H[0, 0] = -dx / r
    H[0, 1] = -dy / r
    H[0, 2] = 0.0
    # bearing derivatives
    H[1, 0] =  dy / q
    H[1, 1] = -dx / q
    H[1, 2] = -1.0
    return H

class EKFLocalizer:
    def __init__(self, x0: np.ndarray, P0: np.ndarray):
        self.x = x0.reshape(3, 1).astype(float)
        self.P = P0.astype(float)

    def predict(self, u: np.ndarray, dt: float, Q: np.ndarray):
        """
        EKF prediction:
        x <- f(x,u)
        P <- F P F^T + L Q L^T
        """
        F = jacobian_F(self.x, u, dt)
        L = jacobian_L(self.x, dt)
        self.x = motion_model(self.x, u, dt)
        self.P = F @ self.P @ F.T + L @ Q @ L.T
        # Enforce symmetry
        self.P = 0.5 * (self.P + self.P.T)

    def update_range_bearing(self, z: np.ndarray, lm: np.ndarray, R: np.ndarray):
        """
        EKF measurement update for range-bearing:
        y = z - h(x)
        S = H P H^T + R
        K = P H^T S^{-1}
        x <- x + K y
        P <- (I - K H) P (I - K H)^T + K R K^T (Joseph form)
        """
        H = jacobian_H_range_bearing(self.x, lm)
        zhat = meas_model_range_bearing(self.x, lm)
        y = z.reshape(2, 1) - zhat
        y[1, 0] = wrap_angle(float(y[1, 0]))  # bearing innovation

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2, 0] = wrap_angle(float(self.x[2, 0]))

        I = np.eye(3, dtype=float)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

def run_demo(seed: int = 7, do_plot: bool = False):
    np.random.seed(seed)

    # Landmarks (world frame)
    landmarks = np.array([
        [5.0,  0.0],
        [5.0,  5.0],
        [0.0,  5.0],
        [-3.0, 2.0],
    ], dtype=float)

    # Simulation parameters
    dt = 0.1
    N = 250

    # True process noise (on control) and assumed process noise in EKF
    sigma_v_true = 0.05
    sigma_w_true = 0.03
    Q = np.diag([0.08**2, 0.05**2])  # assumed (tunable)

    # Measurement noise (range, bearing)
    R = np.diag([0.15**2, (2.0*math.pi/180.0)**2])

    # Initial truth and estimate
    x_true = np.array([[0.0], [0.0], [0.0]])
    x0 = np.array([[-0.2], [0.1], [0.05]])
    P0 = np.diag([0.5**2, 0.5**2, (10.0*math.pi/180.0)**2])

    ekf = EKFLocalizer(x0, P0)

    # Logging
    xs_true = np.zeros((N, 3))
    xs_est = np.zeros((N, 3))

    for k in range(N):
        # Control command (body frame): a gentle curve
        v_cmd = 0.8 + 0.2 * math.sin(0.04 * k)
        w_cmd = 0.25 + 0.05 * math.cos(0.03 * k)
        u_cmd = np.array([[v_cmd], [w_cmd]])

        # Apply noise to control for truth evolution
        u_true = np.array([
            [v_cmd + np.random.normal(0.0, sigma_v_true)],
            [w_cmd + np.random.normal(0.0, sigma_w_true)],
        ], dtype=float)

        # True motion
        x_true = motion_model(x_true, u_true, dt)

        # EKF predict uses commanded control
        ekf.predict(u_cmd, dt, Q)

        # Measurement: pick nearest landmark (simple association)
        dists = np.linalg.norm(landmarks - x_true[:2, 0].reshape(1, 2), axis=1)
        idx = int(np.argmin(dists))
        lm = landmarks[idx]

        z_true = meas_model_range_bearing(x_true, lm)
        z_meas = np.array([
            [z_true[0, 0] + np.random.normal(0.0, math.sqrt(R[0, 0]))],
            [wrap_angle(z_true[1, 0] + np.random.normal(0.0, math.sqrt(R[1, 1])))]
        ], dtype=float)

        # EKF update
        ekf.update_range_bearing(z_meas, lm, R)

        xs_true[k, :] = x_true[:, 0]
        xs_est[k, :] = ekf.x[:, 0]

    # Report RMS error (position + heading)
    pos_err = xs_est[:, :2] - xs_true[:, :2]
    th_err = np.array([wrap_angle(a) for a in (xs_est[:, 2] - xs_true[:, 2])])
    rms_pos = float(np.sqrt(np.mean(np.sum(pos_err**2, axis=1))))
    rms_th = float(np.sqrt(np.mean(th_err**2)))

    print("RMS position error [m]:", rms_pos)
    print("RMS heading error [rad]:", rms_th)

    if do_plot:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(xs_true[:, 0], xs_true[:, 1], label="truth")
        plt.plot(xs_est[:, 0], xs_est[:, 1], label="ekf")
        plt.scatter(landmarks[:, 0], landmarks[:, 1], marker="x", label="landmarks")
        plt.axis("equal")
        plt.legend()
        plt.title("EKF localization demo")
        plt.show()

if __name__ == "__main__":
    run_demo(do_plot=False)
