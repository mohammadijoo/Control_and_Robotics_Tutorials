"""
Chapter7_Lesson4.py
Kalman-Filter Localization for AMR — Lesson 4
Tuning Process/Measurement Noise via Innovation Statistics (NIS) for an EKF.

Dependencies:
  numpy
  scipy (optional, for chi-square thresholds)
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Tuple, List

import numpy as np


@dataclass
class NoiseParams:
    # Control noise std-dev (process noise on controls)
    sigma_v: float       # [m/s]
    sigma_w: float       # [rad/s]
    # GPS measurement noise std-dev
    sigma_gps: float     # [m]
    # Optional scalar inflations (tuning knobs)
    q_scale: float = 1.0
    r_scale: float = 1.0


def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def unicycle_f(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """
    Unicycle kinematics:
      x = [px, py, theta]
      u = [v, w]
    """
    px, py, th = x
    v, w = u
    px2 = px + v * dt * math.cos(th)
    py2 = py + v * dt * math.sin(th)
    th2 = wrap_angle(th + w * dt)
    return np.array([px2, py2, th2], dtype=float)


def jacobian_F(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """Jacobian of f wrt state x."""
    _, _, th = x
    v, _ = u
    F = np.eye(3)
    F[0, 2] = -v * dt * math.sin(th)
    F[1, 2] =  v * dt * math.cos(th)
    return F


def jacobian_L(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """Jacobian of f wrt control noise (v,w)."""
    _, _, th = x
    L = np.zeros((3, 2))
    L[0, 0] = dt * math.cos(th)
    L[1, 0] = dt * math.sin(th)
    L[2, 1] = dt
    return L


def make_Q(x: np.ndarray, u: np.ndarray, dt: float, p: NoiseParams) -> np.ndarray:
    """
    Q = L M L^T, where M is covariance of control noise.
    """
    L = jacobian_L(x, u, dt)
    M = np.diag([p.sigma_v**2, p.sigma_w**2])
    return p.q_scale * (L @ M @ L.T)


def gps_h(x: np.ndarray) -> np.ndarray:
    """GPS measures position only."""
    return x[:2].copy()


def jacobian_H_gps() -> np.ndarray:
    """Jacobian of gps_h wrt state."""
    H = np.zeros((2, 3))
    H[0, 0] = 1.0
    H[1, 1] = 1.0
    return H


def make_R_gps(p: NoiseParams) -> np.ndarray:
    return p.r_scale * np.diag([p.sigma_gps**2, p.sigma_gps**2])


def ekf_predict(x: np.ndarray, P: np.ndarray, u: np.ndarray, dt: float, p: NoiseParams) -> Tuple[np.ndarray, np.ndarray]:
    x_pred = unicycle_f(x, u, dt)
    F = jacobian_F(x, u, dt)
    Q = make_Q(x, u, dt, p)
    P_pred = F @ P @ F.T + Q
    # keep symmetry
    P_pred = 0.5 * (P_pred + P_pred.T)
    return x_pred, P_pred


def ekf_update_gps(x_pred: np.ndarray, P_pred: np.ndarray, z: np.ndarray, p: NoiseParams) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    H = jacobian_H_gps()
    R = make_R_gps(p)
    y = z - gps_h(x_pred)                    # innovation
    S = H @ P_pred @ H.T + R                 # innovation covariance
    K = P_pred @ H.T @ np.linalg.inv(S)      # Kalman gain
    x_upd = x_pred + K @ y
    x_upd[2] = wrap_angle(float(x_upd[2]))
    P_upd = (np.eye(3) - K @ H) @ P_pred
    P_upd = 0.5 * (P_upd + P_upd.T)
    return x_upd, P_upd, y, S


def nis(innovation: np.ndarray, S: np.ndarray) -> float:
    return float(innovation.T @ np.linalg.inv(S) @ innovation)


def nees(x_est: np.ndarray, x_true: np.ndarray, P: np.ndarray) -> float:
    e = x_est - x_true
    e[2] = wrap_angle(float(e[2]))
    return float(e.T @ np.linalg.inv(P) @ e)


def chi2_interval(dof: int, alpha: float = 0.95) -> Tuple[float, float]:
    """
    Two-sided interval for chi-square(dof).
    Requires scipy. If scipy is not available, returns a conservative placeholder.
    """
    try:
        from scipy.stats import chi2
        lo = chi2.ppf((1.0 - alpha) / 2.0, dof)
        hi = chi2.ppf((1.0 + alpha) / 2.0, dof)
        return float(lo), float(hi)
    except Exception:
        # Fallback: very rough (not recommended)
        # For dof=2, 95% interval ~ [0.051, 7.378]
        if dof == 2 and abs(alpha - 0.95) < 1e-9:
            return 0.051, 7.378
        return 0.0, float("inf")


def simulate(T: float = 60.0, dt: float = 0.1, seed: int = 1) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate ground-truth trajectory and noisy controls/GPS.
    Returns:
      x_true[k], u_meas[k], z_gps[k] (gps at each step)
    """
    rng = np.random.default_rng(seed)
    N = int(T / dt)

    x_true = np.zeros((N, 3))
    u_true = np.zeros((N, 2))
    z_gps = np.zeros((N, 2))

    # True controls: piecewise
    for k in range(N):
        t = k * dt
        v = 1.0 + 0.2 * math.sin(0.2 * t)
        w = 0.2 * math.sin(0.1 * t)
        u_true[k] = [v, w]
        if k > 0:
            x_true[k] = unicycle_f(x_true[k - 1], u_true[k - 1], dt)

    # Sensor noise (unknown to filter)
    sigma_v_meas = 0.08
    sigma_w_meas = 0.04
    sigma_gps_meas = 0.6

    u_meas = u_true + rng.normal(0.0, [sigma_v_meas, sigma_w_meas], size=(N, 2))
    z_gps = x_true[:, :2] + rng.normal(0.0, sigma_gps_meas, size=(N, 2))
    return x_true, u_meas, z_gps


def run_filter_and_collect_stats(p: NoiseParams, x_true: np.ndarray, u_meas: np.ndarray, z_gps: np.ndarray, dt: float) -> Tuple[np.ndarray, List[float], List[float]]:
    N = x_true.shape[0]
    x = np.array([0.0, 0.0, 0.0])
    P = np.diag([1.0, 1.0, (10.0 * math.pi / 180.0) ** 2])

    nis_list: List[float] = []
    nees_list: List[float] = []
    x_est_hist = np.zeros_like(x_true)

    for k in range(N):
        x_pred, P_pred = ekf_predict(x, P, u_meas[k], dt, p)
        x, P, innov, S = ekf_update_gps(x_pred, P_pred, z_gps[k], p)
        x_est_hist[k] = x
        nis_list.append(nis(innov, S))
        nees_list.append(nees(x, x_true[k], P))

    return x_est_hist, nis_list, nees_list


def tune_R_scale_by_mean_nis(p: NoiseParams, nis_list: List[float], dof: int) -> NoiseParams:
    """
    Heuristic: if mean(NIS) > dof, R is too small (overconfident measurements) -> increase r_scale.
              if mean(NIS) < dof, R is too large -> decrease r_scale.
    """
    nis_mean = float(np.mean(nis_list))
    # multiplicative update (mild)
    gain = nis_mean / float(dof)
    gain = max(0.2, min(5.0, gain))  # clamp for stability
    p2 = NoiseParams(**{**p.__dict__})
    p2.r_scale *= gain
    return p2


def main() -> None:
    dt = 0.1
    x_true, u_meas, z_gps = simulate(T=60.0, dt=dt, seed=2)

    # Initial guesses (deliberately imperfect)
    p = NoiseParams(sigma_v=0.03, sigma_w=0.01, sigma_gps=0.3, q_scale=1.0, r_scale=1.0)

    # Evaluate and tune
    for it in range(4):
        _, nis_list, nees_list = run_filter_and_collect_stats(p, x_true, u_meas, z_gps, dt)

        dof = 2  # GPS measurement dimension
        lo, hi = chi2_interval(dof=dof, alpha=0.95)
        nis_mean = float(np.mean(nis_list))
        frac_in = float(np.mean((np.array(nis_list) >= lo) & (np.array(nis_list) <= hi)))

        print(f"Iter {it}: r_scale={p.r_scale:.3f}, q_scale={p.q_scale:.3f}")
        print(f"  mean NIS={nis_mean:.3f} (target ~ {dof}), 95% interval [{lo:.3f},{hi:.3f}], fraction-in={frac_in:.3f}")
        print(f"  mean NEES={float(np.mean(nees_list)):.3f} (target ~ state_dim=3)")

        p = tune_R_scale_by_mean_nis(p, nis_list, dof=dof)

    print("Final tuned parameters:", p)


if __name__ == "__main__":
    main()
