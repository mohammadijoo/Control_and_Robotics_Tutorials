#!/usr/bin/env python3
"""
Chapter7_Lesson3.py
Consistency and Linearization Pitfalls — EKF vs UKF on a simple AMR localization example.

Dependencies:
  - numpy
  - scipy (for chi-square bounds)

This script:
  1) Simulates a unicycle-like planar robot (x, y, theta) with process noise.
  2) Uses range-bearing measurements to known landmarks.
  3) Runs EKF and UKF.
  4) Reports ANEES and ANIS consistency statistics with 95% bounds.

Author: (course material)
"""
import math
import numpy as np

try:
    from scipy.stats import chi2
except Exception as e:
    chi2 = None


def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def f_motion(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """Discrete-time motion model (no noise inside; noise added externally)."""
    px, py, th = x
    v, om = u
    nx = px + v * dt * math.cos(th)
    ny = py + v * dt * math.sin(th)
    nth = wrap_angle(th + om * dt)
    return np.array([nx, ny, nth], dtype=float)


def jacobian_F(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """Jacobian of f w.r.t state x at (x,u)."""
    _, _, th = x
    v, _ = u
    F = np.eye(3)
    F[0, 2] = -v * dt * math.sin(th)
    F[1, 2] =  v * dt * math.cos(th)
    return F


def h_meas(x: np.ndarray, landmark: np.ndarray) -> np.ndarray:
    """Range-bearing measurement to a landmark."""
    px, py, th = x
    dx = landmark[0] - px
    dy = landmark[1] - py
    r = math.hypot(dx, dy)
    b = wrap_angle(math.atan2(dy, dx) - th)
    return np.array([r, b], dtype=float)


def jacobian_H(x: np.ndarray, landmark: np.ndarray) -> np.ndarray:
    """Jacobian of h w.r.t state x."""
    px, py, th = x
    dx = landmark[0] - px
    dy = landmark[1] - py
    r2 = dx*dx + dy*dy
    r = math.sqrt(r2) if r2 > 1e-12 else 1e-6
    H = np.zeros((2, 3))
    H[0, 0] = -dx / r
    H[0, 1] = -dy / r
    H[0, 2] = 0.0
    H[1, 0] =  dy / r2
    H[1, 1] = -dx / r2
    H[1, 2] = -1.0
    return H


def ekf_step(x: np.ndarray, P: np.ndarray, u: np.ndarray, z: np.ndarray,
             dt: float, Q: np.ndarray, R: np.ndarray, landmark: np.ndarray):
    """One EKF predict + update."""
    # Predict
    F = jacobian_F(x, u, dt)
    x_pred = f_motion(x, u, dt)
    P_pred = F @ P @ F.T + Q

    # Update
    H = jacobian_H(x_pred, landmark)
    z_pred = h_meas(x_pred, landmark)
    nu = z - z_pred
    nu[1] = wrap_angle(nu[1])  # bearing innovation
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)

    x_upd = x_pred + K @ nu
    x_upd[2] = wrap_angle(x_upd[2])
    I = np.eye(3)
    P_upd = (I - K @ H) @ P_pred @ (I - K @ H).T + K @ R @ K.T  # Joseph form

    return x_upd, P_upd, nu, S


def sigma_points(x: np.ndarray, P: np.ndarray, alpha=1e-3, beta=2.0, kappa=0.0):
    """Scaled unscented sigma points."""
    n = x.size
    lam = alpha*alpha*(n + kappa) - n
    S = np.linalg.cholesky((n + lam) * P)

    X = np.zeros((2*n + 1, n))
    X[0] = x
    for i in range(n):
        X[1 + i]     = x + S[:, i]
        X[1 + n + i] = x - S[:, i]

    Wm = np.full(2*n + 1, 1.0 / (2.0*(n + lam)))
    Wc = np.full(2*n + 1, 1.0 / (2.0*(n + lam)))
    Wm[0] = lam / (n + lam)
    Wc[0] = lam / (n + lam) + (1.0 - alpha*alpha + beta)

    return X, Wm, Wc


def mean_angle(weights: np.ndarray, angles: np.ndarray) -> float:
    s = np.sum(weights * np.sin(angles))
    c = np.sum(weights * np.cos(angles))
    return math.atan2(s, c)


def ukf_step(x: np.ndarray, P: np.ndarray, u: np.ndarray, z: np.ndarray,
             dt: float, Q: np.ndarray, R: np.ndarray, landmark: np.ndarray):
    """One UKF predict + update (state dimension 3, measurement dimension 2)."""
    n = x.size
    m = z.size

    # Predict
    X, Wm, Wc = sigma_points(x, P)
    X_pred = np.zeros_like(X)
    for i in range(X.shape[0]):
        X_pred[i] = f_motion(X[i], u, dt)

    x_pred = np.zeros(n)
    x_pred[0] = np.sum(Wm * X_pred[:, 0])
    x_pred[1] = np.sum(Wm * X_pred[:, 1])
    x_pred[2] = mean_angle(Wm, X_pred[:, 2])

    P_pred = np.zeros((n, n))
    for i in range(X_pred.shape[0]):
        dx = X_pred[i] - x_pred
        dx[2] = wrap_angle(dx[2])
        P_pred += Wc[i] * np.outer(dx, dx)
    P_pred += Q

    # Update
    Zsig = np.zeros((X_pred.shape[0], m))
    for i in range(X_pred.shape[0]):
        Zsig[i] = h_meas(X_pred[i], landmark)

    z_pred = np.zeros(m)
    z_pred[0] = np.sum(Wm * Zsig[:, 0])
    z_pred[1] = mean_angle(Wm, Zsig[:, 1])

    S = np.zeros((m, m))
    Cxz = np.zeros((n, m))
    for i in range(Zsig.shape[0]):
        dz = Zsig[i] - z_pred
        dz[1] = wrap_angle(dz[1])
        dx = X_pred[i] - x_pred
        dx[2] = wrap_angle(dx[2])
        S += Wc[i] * np.outer(dz, dz)
        Cxz += Wc[i] * np.outer(dx, dz)
    S += R

    nu = z - z_pred
    nu[1] = wrap_angle(nu[1])
    K = Cxz @ np.linalg.inv(S)

    x_upd = x_pred + K @ nu
    x_upd[2] = wrap_angle(x_upd[2])
    P_upd = P_pred - K @ S @ K.T

    return x_upd, P_upd, nu, S


def chi2_bounds_for_average(dof: int, N: int, alpha: float = 0.05):
    """Bounds for average of chi-square/dof statistic: (N * avg) ~ chi2(N*dof)."""
    if chi2 is None:
        return None
    lo = chi2.ppf(alpha/2.0, N*dof) / N
    hi = chi2.ppf(1.0 - alpha/2.0, N*dof) / N
    return lo, hi


def run_mc(filter_step, dt: float, steps: int, trials: int, seed: int = 0):
    rng = np.random.default_rng(seed)

    # Landmarks (known map)
    landmarks = np.array([[5.0,  0.0],
                          [0.0,  5.0],
                          [5.0,  5.0]], dtype=float)

    # Noise settings (intentionally a bit "mismatched" to amplify linearization effects)
    Q = np.diag([0.08**2, 0.08**2, (3.0*math.pi/180.0)**2])  # process noise covariance
    R = np.diag([0.15**2, (2.0*math.pi/180.0)**2])          # measurement noise covariance

    nees_list = []
    nis_list = []

    for tr in range(trials):
        # True initial state (unknown to filter)
        x_true = np.array([0.0, 0.0, 0.0], dtype=float)

        # Filter initial belief (purposely slightly biased)
        x_hat = np.array([0.5, -0.4, 10.0*math.pi/180.0], dtype=float)
        P = np.diag([0.8**2, 0.8**2, (15.0*math.pi/180.0)**2])

        for k in range(steps):
            # Control: forward + turning (induces nonlinearity)
            v = 1.0 + 0.2*math.sin(0.05*k)
            om = 0.35 + 0.25*math.sin(0.03*k)

            u = np.array([v, om], dtype=float)

            # Simulate true process with noise
            w = rng.multivariate_normal(np.zeros(3), Q)
            x_true = f_motion(x_true, u, dt) + w
            x_true[2] = wrap_angle(x_true[2])

            # Choose a landmark based on time (switching adds mild nonlinearity)
            lm = landmarks[k % landmarks.shape[0]]

            # Measurement
            z = h_meas(x_true, lm)
            v_meas = rng.multivariate_normal(np.zeros(2), R)
            z = z + v_meas
            z[1] = wrap_angle(z[1])

            # Filter step
            x_hat, P, nu, S = filter_step(x_hat, P, u, z, dt, Q, R, lm)

            # Consistency stats
            e = x_true - x_hat
            e[2] = wrap_angle(e[2])

            nees = float(e.T @ np.linalg.inv(P) @ e)
            nis  = float(nu.T @ np.linalg.inv(S) @ nu)
            nees_list.append(nees)
            nis_list.append(nis)

    nees_arr = np.array(nees_list)
    nis_arr = np.array(nis_list)
    return nees_arr.mean(), nis_arr.mean(), 3, 2, trials*steps


def main():
    # Two dt values: small dt often improves linearization; large dt worsens it
    configs = [
        ("dt=0.05 (mild nonlinearity)", 0.05),
        ("dt=0.20 (stronger nonlinearity)", 0.20),
    ]
    steps = 200
    trials = 40
    alpha = 0.05

    for label, dt in configs:
        print("\n=== ", label, " ===")
        ekf_anees, ekf_anis, n, m, N = run_mc(ekf_step, dt, steps, trials, seed=1)
        ukf_anees, ukf_anis, _, _, _ = run_mc(ukf_step, dt, steps, trials, seed=1)

        print(f"EKF  ANEES={ekf_anees:.3f}   ANIS={ekf_anis:.3f}")
        print(f"UKF  ANEES={ukf_anees:.3f}   ANIS={ukf_anis:.3f}")

        if chi2 is not None:
            nees_lo, nees_hi = chi2_bounds_for_average(n, N, alpha)
            nis_lo, nis_hi = chi2_bounds_for_average(m, N, alpha)
            print(f"95% bounds for ANEES (dof={n}): [{nees_lo:.3f}, {nees_hi:.3f}]")
            print(f"95% bounds for ANIS  (dof={m}): [{nis_lo:.3f}, {nis_hi:.3f}]")
        else:
            print("SciPy not available: chi-square bounds not computed.")

        print("Interpretation: values systematically above bounds indicate overconfidence (inconsistency).")

if __name__ == "__main__":
    main()
