"""
Chapter 11 - SLAM I (Filter-Based SLAM)
Lesson 2: EKF-SLAM (structure and limitations)

Minimal educational EKF-SLAM implementation for a 2D differential-drive robot
with range-bearing measurements to point landmarks.

Assumptions:
- Known data association (landmark id is provided with each measurement).
- Landmarks are static in the world frame.
- Small-world demo with simulated noisy controls + measurements.

Dependencies: numpy
Optional (for plotting): matplotlib
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

import numpy as np


def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Measurement:
    lm_id: int
    rng: float
    brg: float  # bearing in radians (relative to robot heading)


class EKFSLAM:
    """
    State vector:
        x = [x, y, theta, m1x, m1y, m2x, m2y, ...]^T

    Covariance:
        P in R^{(3+2N) x (3+2N)}
    """

    def __init__(self):
        self.mu = np.zeros((3, 1), dtype=float)  # robot pose mean
        self.P = np.eye(3, dtype=float) * 1e-6   # small initial uncertainty
        self.id_to_index: Dict[int, int] = {}    # landmark id -> landmark slot (0..N-1)

    @property
    def n_landmarks(self) -> int:
        return len(self.id_to_index)

    def state_dim(self) -> int:
        return 3 + 2 * self.n_landmarks

    def _lm_slice(self, lm_id: int) -> slice:
        j = self.id_to_index[lm_id]
        start = 3 + 2 * j
        return slice(start, start + 2)

    def _augment_state(self, lm_id: int, m_xy: np.ndarray, P_xm: np.ndarray, P_mm: np.ndarray) -> None:
        """
        Augment mu and P with a new landmark.
        m_xy: (2,1) landmark mean
        P_xm: (D,2) cross-cov between existing state and new landmark
        P_mm: (2,2) landmark covariance
        """
        assert m_xy.shape == (2, 1)
        D = self.mu.shape[0]
        assert P_xm.shape == (D, 2)
        assert P_mm.shape == (2, 2)

        # Augment mean
        self.mu = np.vstack([self.mu, m_xy])

        # Augment covariance
        P_new = np.zeros((D + 2, D + 2), dtype=float)
        P_new[:D, :D] = self.P
        P_new[:D, D:D+2] = P_xm
        P_new[D:D+2, :D] = P_xm.T
        P_new[D:D+2, D:D+2] = P_mm
        self.P = P_new

        # Register landmark
        self.id_to_index[lm_id] = self.n_landmarks  # after augmentation, this is new last slot

    def predict(self, v: float, w: float, dt: float, Q: np.ndarray) -> None:
        """
        Differential-drive / unicycle motion model with control (v, w).
        Q: (2,2) covariance for control noise in [v, w].
        """
        assert Q.shape == (2, 2)
        x, y, th = float(self.mu[0]), float(self.mu[1]), float(self.mu[2])

        # Motion model
        if abs(w) < 1e-9:
            # Straight line
            x_new = x + v * dt * math.cos(th)
            y_new = y + v * dt * math.sin(th)
            th_new = th
        else:
            x_new = x + (v / w) * (math.sin(th + w * dt) - math.sin(th))
            y_new = y - (v / w) * (math.cos(th + w * dt) - math.cos(th))
            th_new = th + w * dt

        th_new = wrap_angle(th_new)

        # Update mean (robot part)
        self.mu[0, 0] = x_new
        self.mu[1, 0] = y_new
        self.mu[2, 0] = th_new

        # Jacobian wrt state (only robot pose affects motion)
        D = self.state_dim()
        F = np.eye(D)

        if abs(w) < 1e-9:
            F[0, 2] = -v * dt * math.sin(th)
            F[1, 2] =  v * dt * math.cos(th)
            # Jacobian wrt controls
            G_u = np.zeros((D, 2))
            G_u[0, 0] = dt * math.cos(th)
            G_u[1, 0] = dt * math.sin(th)
            G_u[2, 1] = dt
        else:
            F[0, 2] = (v / w) * (math.cos(th + w * dt) - math.cos(th))
            F[1, 2] = (v / w) * (math.sin(th + w * dt) - math.sin(th))
            G_u = np.zeros((D, 2))
            # Partial derivatives wrt v and w
            G_u[0, 0] = (1.0 / w) * (math.sin(th + w * dt) - math.sin(th))
            G_u[1, 0] = -(1.0 / w) * (math.cos(th + w * dt) - math.cos(th))
            G_u[2, 1] = dt
            # Derivative wrt w (more accurate linearization)
            G_u[0, 1] = (v / (w * w)) * (math.sin(th) - math.sin(th + w * dt)) + (v / w) * (dt * math.cos(th + w * dt))
            G_u[1, 1] = (v / (w * w)) * (math.cos(th + w * dt) - math.cos(th)) + (v / w) * (dt * math.sin(th + w * dt))

        # Covariance prediction
        self.P = F @ self.P @ F.T + G_u @ Q @ G_u.T

    def _expected_measurement_and_jacobian(self, lm_id: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        For landmark lm_id, compute expected measurement z_hat = [range, bearing]^T
        and Jacobian H wrt full state.
        """
        D = self.state_dim()
        H = np.zeros((2, D), dtype=float)

        x, y, th = float(self.mu[0]), float(self.mu[1]), float(self.mu[2])
        sl = self._lm_slice(lm_id)
        mx, my = float(self.mu[sl.start]), float(self.mu[sl.start + 1])

        dx = mx - x
        dy = my - y
        q = dx * dx + dy * dy
        r = math.sqrt(q)

        z_hat = np.array([[r], [wrap_angle(math.atan2(dy, dx) - th)]], dtype=float)

        # Jacobians
        # range
        H[0, 0] = -dx / r
        H[0, 1] = -dy / r
        H[0, 2] = 0.0
        H[0, sl.start] = dx / r
        H[0, sl.start + 1] = dy / r

        # bearing
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1.0
        H[1, sl.start] = -dy / q
        H[1, sl.start + 1] = dx / q

        return z_hat, H

    def update(self, meas: Measurement, R: np.ndarray) -> None:
        """
        EKF update for an associated landmark. If landmark is unseen, initialize it.
        R: (2,2) measurement noise covariance for [range, bearing]
        """
        assert R.shape == (2, 2)
        if meas.lm_id not in self.id_to_index:
            self._initialize_landmark(meas, R)

        z = np.array([[meas.rng], [wrap_angle(meas.brg)]], dtype=float)
        z_hat, H = self._expected_measurement_and_jacobian(meas.lm_id)

        # Innovation
        y = z - z_hat
        y[1, 0] = wrap_angle(float(y[1, 0]))

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Mean update
        self.mu = self.mu + K @ y
        self.mu[2, 0] = wrap_angle(float(self.mu[2, 0]))

        # Joseph-form covariance update (numerically stable)
        D = self.state_dim()
        I = np.eye(D)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T

        # Symmetrize (protect against numerical drift)
        self.P = 0.5 * (self.P + self.P.T)

    def _initialize_landmark(self, meas: Measurement, R: np.ndarray) -> None:
        """
        Initialize landmark position from the current robot pose and measurement.
        Use inverse observation model:
            m = [x + r cos(theta + b), y + r sin(theta + b)]
        and propagate covariance.
        """
        x, y, th = float(self.mu[0]), float(self.mu[1]), float(self.mu[2])
        r = meas.rng
        b = meas.brg
        ang = th + b

        mx = x + r * math.cos(ang)
        my = y + r * math.sin(ang)
        m = np.array([[mx], [my]], dtype=float)

        # Jacobian of inverse observation wrt robot pose (x,y,th)
        Gx = np.array([
            [1.0, 0.0, -r * math.sin(ang)],
            [0.0, 1.0,  r * math.cos(ang)]
        ], dtype=float)

        # Jacobian wrt measurement (r,b)
        Gz = np.array([
            [math.cos(ang), -r * math.sin(ang)],
            [math.sin(ang),  r * math.cos(ang)]
        ], dtype=float)

        # Existing covariance partitions
        D = self.state_dim()
        P_rr = self.P[:3, :3]  # robot
        P_xr = self.P[:, :3]   # full state vs robot

        P_mm = Gx @ P_rr @ Gx.T + Gz @ R @ Gz.T
        P_xm = P_xr @ Gx.T

        self._augment_state(meas.lm_id, m, P_xm, P_mm)


def simulate_demo(seed: int = 2, steps: int = 200) -> None:
    np.random.seed(seed)

    # Ground-truth map
    landmarks = {
        0: np.array([5.0, 1.0]),
        1: np.array([2.0, 6.0]),
        2: np.array([8.0, 7.0]),
        3: np.array([10.0, 2.5]),
    }

    # Noise models
    Q = np.diag([0.05**2, (math.radians(2.0))**2])   # control noise for v,w
    R = np.diag([0.15**2, (math.radians(3.0))**2])  # measurement noise for range,bearing

    slam = EKFSLAM()
    gt = np.array([0.0, 0.0, 0.0], dtype=float)

    traj_gt = [gt.copy()]
    traj_est = [slam.mu[:3, 0].copy()]

    dt = 0.1
    for k in range(steps):
        # Controls: slightly curving trajectory
        v_cmd = 0.7
        w_cmd = 0.15 * math.sin(0.05 * k)

        # Apply noisy control to ground truth
        v_noisy = v_cmd + np.random.normal(0.0, math.sqrt(Q[0, 0]))
        w_noisy = w_cmd + np.random.normal(0.0, math.sqrt(Q[1, 1]))

        # Propagate ground truth (unicycle)
        x, y, th = gt
        gt[0] = x + v_noisy * dt * math.cos(th)
        gt[1] = y + v_noisy * dt * math.sin(th)
        gt[2] = wrap_angle(th + w_noisy * dt)

        # EKF prediction uses commanded control but with Q
        slam.predict(v_cmd, w_cmd, dt, Q)

        # Generate measurements to nearby landmarks
        for lm_id, lm_xy in landmarks.items():
            dx = lm_xy[0] - gt[0]
            dy = lm_xy[1] - gt[1]
            rng = math.hypot(dx, dy)
            if rng < 7.0:  # sensing range
                brg = wrap_angle(math.atan2(dy, dx) - gt[2])
                rng_m = rng + np.random.normal(0.0, math.sqrt(R[0, 0]))
                brg_m = wrap_angle(brg + np.random.normal(0.0, math.sqrt(R[1, 1])))
                slam.update(Measurement(lm_id, rng_m, brg_m), R)

        traj_gt.append(gt.copy())
        traj_est.append(slam.mu[:3, 0].copy())

    traj_gt = np.array(traj_gt)
    traj_est = np.array(traj_est)

    # Print a quick summary
    pos_err = np.linalg.norm(traj_gt[-1, :2] - traj_est[-1, :2])
    ang_err = abs(wrap_angle(traj_gt[-1, 2] - traj_est[-1, 2]))
    print("Final position error:", pos_err)
    print("Final angle error (deg):", math.degrees(ang_err))
    print("Estimated landmarks:", slam.n_landmarks)
    for lm_id in sorted(slam.id_to_index.keys()):
        sl = slam._lm_slice(lm_id)
        est = slam.mu[sl, 0]
        print(f"  lm {lm_id}: est={est}, true={landmarks[lm_id]}")

    # Optional plotting
    try:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(traj_gt[:, 0], traj_gt[:, 1], label="ground truth")
        plt.plot(traj_est[:, 0], traj_est[:, 1], label="EKF-SLAM estimate")
        for lm_id, lm_xy in landmarks.items():
            plt.scatter([lm_xy[0]], [lm_xy[1]], marker="x")
        plt.axis("equal")
        plt.legend()
        plt.title("EKF-SLAM demo (known association)")
        plt.show()
    except Exception as e:
        print("Plotting skipped:", e)


if __name__ == "__main__":
    simulate_demo()
