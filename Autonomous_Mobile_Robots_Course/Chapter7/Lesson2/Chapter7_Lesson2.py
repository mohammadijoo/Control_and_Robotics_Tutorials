"""
Chapter7_Lesson2.py
UKF localization for a planar (x, y, theta) mobile robot with range-bearing landmarks.

Dependencies:
  - numpy
  - matplotlib (optional, for plotting)

This script implements:
  (1) A minimal Unscented Kalman Filter (UKF) from scratch (additive noise form).
  (2) A minimal Extended Kalman Filter (EKF) baseline for comparison.
  (3) A small simulation: differential-drive kinematics with noisy range-bearing measurements.

Author: Course material generator
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Tuple

import numpy as np


def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return float(math.atan2(math.sin(a), math.cos(a)))


def mean_angle(angles: np.ndarray, w: np.ndarray) -> float:
    """Weighted circular mean."""
    s = float(np.sum(w * np.sin(angles)))
    c = float(np.sum(w * np.cos(angles)))
    return wrap_angle(math.atan2(s, c))


def block_diag(*mats):
    """Simple block diagonal builder."""
    n = sum(m.shape[0] for m in mats)
    out = np.zeros((n, n))
    i = 0
    for m in mats:
        r = m.shape[0]
        out[i:i+r, i:i+r] = m
        i += r
    return out


@dataclass
class UKFParams:
    alpha: float = 0.3   # small positive (0, 1]
    beta: float = 2.0    # 2 is optimal for Gaussian priors
    kappa: float = 0.0   # often 0


class UKF:
    """
    Additive-noise UKF:
      x_k+1 = f(x_k, u_k) + w_k,  w ~ N(0, Q)
      z_k   = h(x_k)      + v_k,  v ~ N(0, R)

    State: x = [x, y, theta]^T (theta is angular; we handle it with circular mean).
    """

    def __init__(self, dim_x: int, params: UKFParams):
        self.n = dim_x
        self.params = params
        self.lmbda = params.alpha**2 * (self.n + params.kappa) - self.n

        # Weights for mean/covariance
        self.Wm = np.full(2 * self.n + 1, 1.0 / (2.0 * (self.n + self.lmbda)))
        self.Wc = self.Wm.copy()
        self.Wm[0] = self.lmbda / (self.n + self.lmbda)
        self.Wc[0] = self.Wm[0] + (1.0 - params.alpha**2 + params.beta)

        self.x = np.zeros(self.n)
        self.P = np.eye(self.n)

        self.Q = np.eye(self.n) * 1e-3  # process noise (to be set)
        # NOTE: measurement noise R is set per update (since we do sequential landmark updates)

    def sigma_points(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        """Compute sigma points matrix [2n+1, n]."""
        n = self.n
        S = np.linalg.cholesky((n + self.lmbda) * P)
        X = np.zeros((2 * n + 1, n))
        X[0] = x
        for i in range(n):
            X[i + 1] = x + S[:, i]
            X[i + 1 + n] = x - S[:, i]
        # wrap theta component of sigma points (index 2) for numerical stability
        X[:, 2] = np.vectorize(wrap_angle)(X[:, 2])
        return X

    def _state_mean(self, X: np.ndarray) -> np.ndarray:
        """Weighted mean for state with angular component theta at index 2."""
        x_mean = np.zeros(self.n)
        # linear components
        x_mean[0] = float(np.sum(self.Wm * X[:, 0]))
        x_mean[1] = float(np.sum(self.Wm * X[:, 1]))
        # angular component
        x_mean[2] = mean_angle(X[:, 2], self.Wm)
        return x_mean

    def _state_residual(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """a - b with angle wrap on theta."""
        y = a - b
        y[2] = wrap_angle(float(y[2]))
        return y

    def predict(self, f: Callable[[np.ndarray, np.ndarray], np.ndarray], u: np.ndarray) -> None:
        """Time update."""
        X = self.sigma_points(self.x, self.P)

        # propagate sigma points
        Xp = np.zeros_like(X)
        for i in range(X.shape[0]):
            Xp[i] = f(X[i], u)
            Xp[i, 2] = wrap_angle(float(Xp[i, 2]))

        x_pred = self._state_mean(Xp)

        P_pred = np.zeros((self.n, self.n))
        for i in range(Xp.shape[0]):
            dx = self._state_residual(Xp[i], x_pred).reshape(-1, 1)
            P_pred += self.Wc[i] * (dx @ dx.T)
        P_pred += self.Q

        self.x, self.P = x_pred, P_pred

    def update_sequential(
        self,
        z: np.ndarray,
        h: Callable[[np.ndarray], np.ndarray],
        R: np.ndarray,
        angle_index: int = 1,
    ) -> None:
        """
        Measurement update for a 2D measurement z=[range, bearing] (bearing is angular).
        angle_index: index of angular component in measurement (bearing=1).
        """
        X = self.sigma_points(self.x, self.P)

        # propagate to measurement space
        Z = np.zeros((X.shape[0], z.shape[0]))
        for i in range(X.shape[0]):
            Z[i] = h(X[i])
            Z[i, angle_index] = wrap_angle(float(Z[i, angle_index]))

        # mean measurement (circular mean on bearing)
        z_pred = np.zeros(z.shape[0])
        z_pred[0] = float(np.sum(self.Wm * Z[:, 0]))
        z_pred[1] = mean_angle(Z[:, 1], self.Wm)

        # innovation covariance S and cross covariance Pxz
        S = np.zeros((z.shape[0], z.shape[0]))
        Pxz = np.zeros((self.n, z.shape[0]))
        for i in range(Z.shape[0]):
            dz = (Z[i] - z_pred).reshape(-1, 1)
            dz[angle_index, 0] = wrap_angle(float(dz[angle_index, 0]))
            dx = self._state_residual(X[i], self.x).reshape(-1, 1)
            S += self.Wc[i] * (dz @ dz.T)
            Pxz += self.Wc[i] * (dx @ dz.T)
        S += R

        # Kalman gain and update
        K = Pxz @ np.linalg.inv(S)
        innov = z - z_pred
        innov[angle_index] = wrap_angle(float(innov[angle_index]))

        self.x = self.x + K @ innov
        self.x[2] = wrap_angle(float(self.x[2]))
        self.P = self.P - K @ S @ K.T


class EKF:
    """Minimal EKF baseline for the same system."""

    def __init__(self):
        self.x = np.zeros(3)
        self.P = np.eye(3)
        self.Q = np.eye(3) * 1e-3

    def predict(self, u: np.ndarray, dt: float):
        x, y, th = self.x
        v, w = u
        # nonlinear motion
        x2 = x + dt * v * math.cos(th)
        y2 = y + dt * v * math.sin(th)
        th2 = wrap_angle(th + dt * w)
        self.x = np.array([x2, y2, th2])

        # Jacobian F = df/dx
        F = np.array([
            [1.0, 0.0, -dt * v * math.sin(th)],
            [0.0, 1.0,  dt * v * math.cos(th)],
            [0.0, 0.0, 1.0],
        ])
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z: np.ndarray, landmark: np.ndarray, R: np.ndarray):
        x, y, th = self.x
        lx, ly = landmark
        dx = lx - x
        dy = ly - y
        r = math.sqrt(dx*dx + dy*dy)
        b = wrap_angle(math.atan2(dy, dx) - th)
        z_pred = np.array([r, b])

        # Jacobian H = dh/dx
        # r = sqrt(dx^2+dy^2)
        # b = atan2(dy,dx) - th
        eps = 1e-9
        r2 = max(r*r, eps)
        H = np.array([
            [-dx / max(r, eps), -dy / max(r, eps), 0.0],
            [ dy / r2,         -dx / r2,         -1.0],
        ])

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        innov = z - z_pred
        innov[1] = wrap_angle(float(innov[1]))

        self.x = self.x + K @ innov
        self.x[2] = wrap_angle(float(self.x[2]))
        self.P = (np.eye(3) - K @ H) @ self.P


def motion_model(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """Differential-drive kinematic model in SE(2) coordinates (pose)."""
    px, py, th = x
    v, w = u
    return np.array([
        px + dt * v * math.cos(th),
        py + dt * v * math.sin(th),
        wrap_angle(th + dt * w),
    ])


def measurement_model_factory(landmark: np.ndarray):
    """Returns h(x) for a fixed landmark."""
    lx, ly = float(landmark[0]), float(landmark[1])

    def h(x: np.ndarray) -> np.ndarray:
        px, py, th = float(x[0]), float(x[1]), float(x[2])
        dx = lx - px
        dy = ly - py
        r = math.sqrt(dx*dx + dy*dy)
        b = wrap_angle(math.atan2(dy, dx) - th)
        return np.array([r, b])

    return h


def run_demo(seed: int = 7):
    np.random.seed(seed)

    dt = 0.1
    T = 250

    # landmarks in the plane
    landmarks = np.array([
        [5.0,  0.0],
        [0.0,  6.0],
        [6.0,  6.0],
        [8.0, -2.0],
    ])

    # true state
    x_true = np.array([0.0, 0.0, 0.2])

    # filter init
    ukf = UKF(dim_x=3, params=UKFParams(alpha=0.35, beta=2.0, kappa=0.0))
    ukf.x = np.array([0.5, -0.5, -0.3])
    ukf.P = np.diag([0.8**2, 0.8**2, (20.0 * math.pi/180.0)**2])

    ekf = EKF()
    ekf.x = ukf.x.copy()
    ekf.P = ukf.P.copy()

    # noise models
    sigma_xy = 0.02
    sigma_th = math.radians(1.0)
    ukf.Q = np.diag([sigma_xy**2, sigma_xy**2, sigma_th**2])
    ekf.Q = ukf.Q.copy()

    sigma_r = 0.15
    sigma_b = math.radians(2.0)
    R = np.diag([sigma_r**2, sigma_b**2])

    xs_true, xs_ukf, xs_ekf = [], [], []

    for k in range(T):
        # time-varying control input (moderate nonlinearity)
        v = 1.0 + 0.2 * math.sin(0.07 * k)
        w = 0.35 * math.sin(0.03 * k)  # turning

        u = np.array([v, w])

        # simulate true motion with process noise
        x_true = motion_model(x_true, u, dt)
        x_true += np.array([
            np.random.normal(0, sigma_xy),
            np.random.normal(0, sigma_xy),
            np.random.normal(0, sigma_th),
        ])
        x_true[2] = wrap_angle(float(x_true[2]))

        # UKF predict
        ukf.predict(lambda x, uu: motion_model(x, uu, dt), u)
        # EKF predict
        ekf.predict(u, dt)

        # generate measurements to all landmarks and apply sequential updates
        for lm in landmarks:
            h = measurement_model_factory(lm)
            z = h(x_true) + np.array([
                np.random.normal(0, sigma_r),
                np.random.normal(0, sigma_b),
            ])
            z[1] = wrap_angle(float(z[1]))

            ukf.update_sequential(z, h, R, angle_index=1)
            ekf.update(z, lm, R)

        xs_true.append(x_true.copy())
        xs_ukf.append(ukf.x.copy())
        xs_ekf.append(ekf.x.copy())

    xs_true = np.array(xs_true)
    xs_ukf = np.array(xs_ukf)
    xs_ekf = np.array(xs_ekf)

    pos_err_ukf = np.linalg.norm(xs_ukf[:, :2] - xs_true[:, :2], axis=1)
    pos_err_ekf = np.linalg.norm(xs_ekf[:, :2] - xs_true[:, :2], axis=1)

    def ang_err(est, tru):
        return np.array([wrap_angle(float(e - t)) for e, t in zip(est, tru)])

    th_err_ukf = np.abs(ang_err(xs_ukf[:, 2], xs_true[:, 2]))
    th_err_ekf = np.abs(ang_err(xs_ekf[:, 2], xs_true[:, 2]))

    print("Final true state:", xs_true[-1])
    print("Final UKF  state:", xs_ukf[-1])
    print("Final EKF  state:", xs_ekf[-1])

    print("\nRMSE position (UKF):", float(np.sqrt(np.mean(pos_err_ukf**2))))
    print("RMSE position (EKF):", float(np.sqrt(np.mean(pos_err_ekf**2))))
    print("RMSE theta    (UKF):", float(np.sqrt(np.mean(th_err_ukf**2))))
    print("RMSE theta    (EKF):", float(np.sqrt(np.mean(th_err_ekf**2))))

    # Optional plot
    try:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(xs_true[:, 0], xs_true[:, 1], label="true")
        plt.plot(xs_ukf[:, 0], xs_ukf[:, 1], label="ukf")
        plt.plot(xs_ekf[:, 0], xs_ekf[:, 1], label="ekf", linestyle="--")
        plt.scatter(landmarks[:, 0], landmarks[:, 1], marker="x", label="landmarks")
        plt.axis("equal")
        plt.legend()
        plt.title("UKF vs EKF localization (range-bearing landmarks)")
        plt.show()
    except Exception as e:
        print("Plot skipped:", e)


if __name__ == "__main__":
    run_demo()
