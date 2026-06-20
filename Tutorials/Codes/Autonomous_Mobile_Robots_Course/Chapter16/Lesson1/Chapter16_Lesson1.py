"""
Chapter16_Lesson1.py
Sensing Dynamic Obstacles — Detection + Multi-Target Tracking (Teaching Baseline)

This script demonstrates:
1) Synthetic moving obstacles (2D constant-velocity model)
2) Noisy detections with clutter
3) Multi-target tracking with: KF (per track) + chi-square gating + greedy NN association

Dependencies: numpy, matplotlib
Optional: scipy (for chi-square inverse CDF) — if not available, uses a small lookup.
"""

from __future__ import annotations
import math
import numpy as np
import matplotlib.pyplot as plt


def chi2_inv_cdf(p: float, dof: int) -> float:
    """
    Approximate inverse CDF of chi-square for teaching/demo.
    If SciPy is available, use scipy.stats.chi2.ppf.
    """
    try:
        from scipy.stats import chi2  # type: ignore
        return float(chi2.ppf(p, dof))
    except Exception:
        # Coarse fallback table for dof=2,4 at common probabilities (pg)
        table = {
            (2, 0.90): 4.605,
            (2, 0.95): 5.991,
            (2, 0.97): 7.378,
            (2, 0.99): 9.210,
            (4, 0.90): 7.779,
            (4, 0.95): 9.488,
            (4, 0.97): 11.143,
            (4, 0.99): 13.277,
        }
        key = (dof, round(p, 2))
        if key in table:
            return table[key]
        # Very rough Wilson–Hilferty approximation
        # X ~ chi2_k => (X/k)^(1/3) approx N(1-2/(9k), 2/(9k))
        k = float(dof)
        z = math.sqrt(2) * erfinv(2 * p - 1)  # approx normal quantile
        return k * (1 - 2/(9*k) + z * math.sqrt(2/(9*k)))**3


def erfinv(x: float) -> float:
    """Approximate inverse error function (for fallback chi-square)."""
    # Winitzki approximation
    a = 0.147
    s = 1.0 if x >= 0 else -1.0
    ln = math.log(1.0 - x * x)
    t = 2.0/(math.pi*a) + ln/2.0
    return s * math.sqrt(math.sqrt(t*t - ln/a) - t)


def make_F_Q(dt: float, sigma_a: float) -> tuple[np.ndarray, np.ndarray]:
    F = np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ], dtype=float)

    q = sigma_a**2
    Q = q * np.array([
        [dt**4/4, 0,        dt**3/2, 0],
        [0,       dt**4/4,  0,       dt**3/2],
        [dt**3/2, 0,        dt**2,   0],
        [0,       dt**3/2,  0,       dt**2],
    ], dtype=float)
    return F, Q


def kf_predict(x: np.ndarray, P: np.ndarray, F: np.ndarray, Q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    x2 = F @ x
    P2 = F @ P @ F.T + Q
    return x2, P2


def kf_update(x: np.ndarray, P: np.ndarray, z: np.ndarray, H: np.ndarray, R: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)

    x2 = x + K @ y
    I = np.eye(P.shape[0])
    # Joseph form for numerical stability
    P2 = (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T

    d2 = float(y.T @ np.linalg.inv(S) @ y)  # Mahalanobis distance squared
    return x2, P2, d2


class Track:
    def __init__(self, x: np.ndarray, P: np.ndarray, track_id: int):
        self.x = x.copy()
        self.P = P.copy()
        self.id = track_id
        self.age = 1
        self.hits = 1
        self.misses = 0

    def predict(self, F: np.ndarray, Q: np.ndarray):
        self.x, self.P = kf_predict(self.x, self.P, F, Q)
        self.age += 1

    def update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray):
        self.x, self.P, _ = kf_update(self.x, self.P, z, H, R)
        self.hits += 1
        self.misses = 0


def greedy_nn_association(tracks: list[Track], Z: np.ndarray, H: np.ndarray, R: np.ndarray, gate_gamma: float) -> tuple[list[tuple[int,int]], list[int], list[int]]:
    """
    Return matched pairs (track_index, meas_index), unmatched_tracks, unmatched_meas.
    Greedy NN on gated Mahalanobis distance (teaching baseline).
    """
    if len(tracks) == 0:
        return [], [], list(range(len(Z)))
    if len(Z) == 0:
        return [], list(range(len(tracks))), []

    pairs = []
    for ti, trk in enumerate(tracks):
        # Innovation covariance S for each track (same H, R)
        S = H @ trk.P @ H.T + R
        S_inv = np.linalg.inv(S)
        zhat = H @ trk.x
        for mi, z in enumerate(Z):
            y = z - zhat
            d2 = float(y.T @ S_inv @ y)
            if d2 <= gate_gamma:
                pairs.append((d2, ti, mi))

    pairs.sort(key=lambda t: t[0])

    matched_tracks = set()
    matched_meas = set()
    matches: list[tuple[int,int]] = []
    for d2, ti, mi in pairs:
        if ti in matched_tracks or mi in matched_meas:
            continue
        matched_tracks.add(ti)
        matched_meas.add(mi)
        matches.append((ti, mi))

    unmatched_tracks = [i for i in range(len(tracks)) if i not in matched_tracks]
    unmatched_meas = [i for i in range(len(Z)) if i not in matched_meas]
    return matches, unmatched_tracks, unmatched_meas


def simulate_truth(num_targets: int, T: int, dt: float, area: float, seed: int = 0) -> np.ndarray:
    """
    Truth states: shape (T, num_targets, 4) for [px,py,vx,vy]
    """
    rng = np.random.default_rng(seed)
    X = np.zeros((T, num_targets, 4), dtype=float)
    # Random initial positions and velocities
    X[0, :, 0:2] = rng.uniform(-area, area, size=(num_targets, 2))
    X[0, :, 2:4] = rng.uniform(-1.2, 1.2, size=(num_targets, 2))
    for k in range(1, T):
        X[k, :, 0:2] = X[k-1, :, 0:2] + dt * X[k-1, :, 2:4]
        X[k, :, 2:4] = X[k-1, :, 2:4]
        # bounce on boundaries
        for j in range(num_targets):
            for ax in [0, 1]:
                if abs(X[k, j, ax]) > area:
                    X[k, j, ax] = np.clip(X[k, j, ax], -area, area)
                    X[k, j, ax+2] *= -1.0
    return X


def generate_measurements(truth: np.ndarray, sigma_z: float, p_detect: float, clutter_rate: int, area: float, seed: int = 1) -> list[np.ndarray]:
    """
    For each time k, return array Z_k of detections [px,py] with noise, missed detections, and uniform clutter.
    """
    rng = np.random.default_rng(seed)
    T, N, _ = truth.shape
    Zs: list[np.ndarray] = []
    for k in range(T):
        meas = []
        # true detections
        for j in range(N):
            if rng.uniform() <= p_detect:
                z = truth[k, j, 0:2] + rng.normal(0.0, sigma_z, size=(2,))
                meas.append(z)
        # clutter (false alarms)
        m_clutter = rng.poisson(clutter_rate)
        for _ in range(m_clutter):
            zc = rng.uniform(-area, area, size=(2,))
            meas.append(zc)
        if len(meas) == 0:
            Zs.append(np.zeros((0, 2)))
        else:
            Zs.append(np.vstack(meas))
    return Zs


def main():
    # Scenario
    dt = 0.1
    T = 220
    area = 10.0

    num_targets = 3
    sigma_a = 0.7          # process accel std (model)
    sigma_z = 0.35         # measurement std
    p_detect = 0.92
    clutter_rate = 2       # expected clutter per frame

    # Filter/association parameters
    gate_pg = 0.99
    gate_gamma = chi2_inv_cdf(gate_pg, dof=2)  # position measurement => m=2
    max_misses = 12
    init_P = np.diag([1.0, 1.0, 2.0, 2.0])

    F, Q = make_F_Q(dt, sigma_a)
    H = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0]], dtype=float)
    R = (sigma_z**2) * np.eye(2)

    truth = simulate_truth(num_targets, T, dt, area, seed=2)
    Zs = generate_measurements(truth, sigma_z, p_detect, clutter_rate, area, seed=3)

    tracks: list[Track] = []
    next_id = 1

    # For plotting
    est_hist = []  # list of (k, track_id, px, py)
    det_hist = []  # list of (k, px, py)

    for k in range(T):
        Zk = Zs[k]
        for z in Zk:
            det_hist.append((k, float(z[0]), float(z[1])))

        # Predict all tracks
        for trk in tracks:
            trk.predict(F, Q)

        # Associate
        matches, um_tracks, um_meas = greedy_nn_association(tracks, Zk, H, R, gate_gamma)

        # Update matched
        for ti, mi in matches:
            tracks[ti].update(Zk[mi], H, R)

        # Mark misses
        for ti in um_tracks:
            tracks[ti].misses += 1

        # Spawn new tracks for unmatched measurements (simple: use position, zero velocity)
        for mi in um_meas:
            z = Zk[mi]
            x0 = np.array([z[0], z[1], 0.0, 0.0], dtype=float)
            tracks.append(Track(x0, init_P, next_id))
            next_id += 1

        # Prune
        tracks = [t for t in tracks if t.misses <= max_misses]

        # Store estimates
        for trk in tracks:
            est_hist.append((k, trk.id, float(trk.x[0]), float(trk.x[1])))

    # Plot (truth, detections, track estimates)
    plt.figure(figsize=(9, 7))
    # truth
    for j in range(num_targets):
        plt.plot(truth[:, j, 0], truth[:, j, 1], linewidth=2, label=f"truth {j+1}")
    # detections (subsample to reduce clutter)
    det_xy = np.array([(x, y) for (_, x, y) in det_hist], dtype=float)
    if det_xy.size > 0:
        plt.scatter(det_xy[::3, 0], det_xy[::3, 1], s=12, alpha=0.30, label="detections (subsample)")

    # estimates by track id
    est = np.array(est_hist, dtype=float)
    if est.size > 0:
        ids = np.unique(est[:, 1]).astype(int)
        for tid in ids:
            mask = (est[:, 1].astype(int) == tid)
            plt.plot(est[mask, 2], est[mask, 3], linestyle="--", linewidth=1)

    plt.axis("equal")
    plt.xlim(-area-1, area+1)
    plt.ylim(-area-1, area+1)
    plt.title("Dynamic obstacle sensing demo: detections + multi-target tracking")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend(loc="upper right", fontsize=9)
    plt.grid(True)
    plt.show()

    print("Done.")
    print(f"Gate (pg={gate_pg}) gamma={gate_gamma:.3f}, max_misses={max_misses}, tracks_final={len(tracks)}")


if __name__ == "__main__":
    main()
