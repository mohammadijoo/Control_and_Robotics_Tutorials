# Chapter11_Lesson3.py
"""
FastSLAM 1.0 (Rao–Blackwellized Particle Filter SLAM) — minimal educational implementation.

Assumptions (kept simple for teaching):
- 2D planar robot with state x = [px, py, theta]
- Known data association (each observation includes landmark id)
- Range-bearing landmark sensor with Gaussian noise
- Motion model: unicycle with noisy controls

This script:
1) Simulates a small world with point landmarks.
2) Runs FastSLAM 1.0 with N particles, each maintaining per-landmark EKF.
3) Plots ground-truth and estimated trajectory.

Dependencies: numpy, matplotlib (optional for plotting)
"""

from __future__ import annotations
import math
import numpy as np

try:
    import matplotlib.pyplot as plt
    _HAS_PLT = True
except Exception:
    _HAS_PLT = False

def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a

def motion_model(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """Deterministic unicycle step."""
    px, py, th = x
    v, w = u
    if abs(w) < 1e-9:
        px2 = px + v * dt * math.cos(th)
        py2 = py + v * dt * math.sin(th)
        th2 = th
    else:
        px2 = px + (v / w) * (math.sin(th + w * dt) - math.sin(th))
        py2 = py - (v / w) * (math.cos(th + w * dt) - math.cos(th))
        th2 = th + w * dt
    return np.array([px2, py2, wrap_angle(th2)], dtype=float)

def noisy_motion(x: np.ndarray, u: np.ndarray, dt: float, R_u: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Sample a motion step by perturbing controls (v,w)."""
    u_noisy = u + rng.multivariate_normal(mean=np.zeros(2), cov=R_u)
    return motion_model(x, u_noisy, dt)

def h_landmark(x: np.ndarray, m: np.ndarray) -> np.ndarray:
    """Range-bearing measurement model to landmark m in world frame."""
    px, py, th = x
    dx = m[0] - px
    dy = m[1] - py
    q = dx*dx + dy*dy
    r = math.sqrt(q)
    b = wrap_angle(math.atan2(dy, dx) - th)
    return np.array([r, b], dtype=float)

def H_landmark(x: np.ndarray, m: np.ndarray) -> np.ndarray:
    """Jacobian dh/dm for landmark position m=[mx,my]."""
    px, py, th = x
    dx = m[0] - px
    dy = m[1] - py
    q = dx*dx + dy*dy
    r = math.sqrt(q)
    # Avoid division by zero
    q = max(q, 1e-12)
    r = max(r, 1e-6)
    return np.array([[dx/r, dy/r],
                     [-dy/q, dx/q]], dtype=float)

def inv_measurement(x: np.ndarray, z: np.ndarray) -> np.ndarray:
    """Inverse sensor model: given pose x and measurement z=[r,b], return landmark position."""
    px, py, th = x
    r, b = float(z[0]), float(z[1])
    ang = th + b
    mx = px + r * math.cos(ang)
    my = py + r * math.sin(ang)
    return np.array([mx, my], dtype=float)

def gaussian_likelihood(v: np.ndarray, S: np.ndarray) -> float:
    """Evaluate N(v;0,S) with small numerical safeguards."""
    # 2D only
    S = 0.5 * (S + S.T)
    detS = float(np.linalg.det(S))
    detS = max(detS, 1e-18)
    invS = np.linalg.inv(S)
    e = float(v.T @ invS @ v)
    norm = 1.0 / (2.0 * math.pi * math.sqrt(detS))
    return norm * math.exp(-0.5 * e)

class LandmarkEKF:
    def __init__(self):
        self.seen = False
        self.mu = np.zeros(2, dtype=float)
        self.Sigma = np.eye(2, dtype=float) * 1e6

class Particle:
    def __init__(self, n_landmarks: int):
        self.x = np.zeros(3, dtype=float)
        self.w = 1.0
        self.lms = [LandmarkEKF() for _ in range(n_landmarks)]

def systematic_resample(particles: list[Particle], rng: np.random.Generator) -> list[Particle]:
    """Systematic resampling on normalized weights."""
    N = len(particles)
    ws = np.array([p.w for p in particles], dtype=float)
    ws = ws / max(ws.sum(), 1e-12)
    cdf = np.cumsum(ws)

    u0 = rng.uniform(0.0, 1.0 / N)
    idxs = []
    j = 0
    for i in range(N):
        u = u0 + i / N
        while u > cdf[j]:
            j += 1
            if j >= N:
                j = N - 1
                break
        idxs.append(j)

    # Deep copy particles (including landmark EKFs)
    new_ps = []
    for j in idxs:
        pj = particles[j]
        pnew = Particle(len(pj.lms))
        pnew.x = pj.x.copy()
        pnew.w = 1.0 / N
        for k, lm in enumerate(pj.lms):
            pnew.lms[k].seen = lm.seen
            pnew.lms[k].mu = lm.mu.copy()
            pnew.lms[k].Sigma = lm.Sigma.copy()
        new_ps.append(pnew)
    return new_ps

def fastslam_1_step(particles: list[Particle],
                    u: np.ndarray,
                    z_list: list[tuple[int, np.ndarray]],
                    dt: float,
                    R_u: np.ndarray,
                    Q_z: np.ndarray,
                    rng: np.random.Generator) -> list[Particle]:
    """
    One FastSLAM 1.0 step:
    1) sample motion for each particle
    2) per particle: update per-landmark EKFs, and multiply importance weight by measurement likelihood
    3) normalize + resample if ESS low
    """
    N = len(particles)

    # 1) motion sampling
    for p in particles:
        p.x = noisy_motion(p.x, u, dt, R_u, rng)

    # 2) measurement updates
    for p in particles:
        for lm_id, z in z_list:
            lm = p.lms[lm_id]
            if not lm.seen:
                # initialize landmark EKF from inverse measurement
                lm.mu = inv_measurement(p.x, z)
                H = H_landmark(p.x, lm.mu)
                # Sigma = (H^{-1}) Q (H^{-1})^T
                J = np.linalg.inv(H)
                lm.Sigma = J @ Q_z @ J.T
                lm.seen = True
                # initialization likelihood (optional). We keep neutral weight here.
                continue

            zhat = h_landmark(p.x, lm.mu)
            v = np.array([z[0] - zhat[0], wrap_angle(z[1] - zhat[1])], dtype=float)
            H = H_landmark(p.x, lm.mu)
            S = H @ lm.Sigma @ H.T + Q_z
            K = lm.Sigma @ H.T @ np.linalg.inv(S)
            lm.mu = lm.mu + K @ v
            lm.Sigma = (np.eye(2) - K @ H) @ lm.Sigma

            # importance weight multiplier
            p.w *= gaussian_likelihood(v, S)

    # 3) normalize
    ws = np.array([p.w for p in particles], dtype=float)
    s = float(ws.sum())
    if s <= 1e-30:
        # catastrophic degeneracy; reset to uniform
        for p in particles:
            p.w = 1.0 / N
    else:
        for p in particles:
            p.w /= s

    # effective sample size
    ws = np.array([p.w for p in particles], dtype=float)
    ess = 1.0 / float(np.sum(ws * ws))
    if ess < 0.5 * N:
        particles = systematic_resample(particles, rng)

    return particles

def estimate_pose(particles: list[Particle]) -> np.ndarray:
    """Weighted mean pose (angle via circular mean)."""
    ws = np.array([p.w for p in particles], dtype=float)
    ws = ws / max(ws.sum(), 1e-12)
    xs = np.array([p.x for p in particles], dtype=float)
    px = float(np.sum(ws * xs[:, 0]))
    py = float(np.sum(ws * xs[:, 1]))
    # circular mean for theta
    c = float(np.sum(ws * np.cos(xs[:, 2])))
    s = float(np.sum(ws * np.sin(xs[:, 2])))
    th = math.atan2(s, c)
    return np.array([px, py, th], dtype=float)

def main():
    rng = np.random.default_rng(42)

    # World (landmarks)
    landmarks = np.array([[2.0, 2.0],
                          [8.0, 1.0],
                          [6.5, 7.0],
                          [1.0, 8.0],
                          [9.0, 9.0]], dtype=float)
    M = landmarks.shape[0]

    # Simulation settings
    T = 150
    dt = 0.1
    max_range = 7.0

    # True motion & sensor noise
    R_u = np.diag([0.05**2, (2.0*math.pi/180.0)**2])  # noise on [v,w]
    Q_z = np.diag([0.15**2, (3.0*math.pi/180.0)**2])  # noise on [range,bearing]

    # Control sequence: gentle turns
    controls = []
    for t in range(T):
        v = 0.8
        w = 0.25 * math.sin(0.05 * t)
        controls.append(np.array([v, w], dtype=float))

    # Generate ground truth + observations
    x_true = np.array([0.0, 0.0, 0.0], dtype=float)
    xs_true = [x_true.copy()]
    Z = []  # list per time: [(id, z), ...]
    for t in range(T):
        x_true = noisy_motion(x_true, controls[t], dt, R_u, rng)  # true evolves with same noise class
        xs_true.append(x_true.copy())

        z_list = []
        for i in range(M):
            zhat = h_landmark(x_true, landmarks[i])
            if zhat[0] <= max_range:
                z = zhat + rng.multivariate_normal(mean=np.zeros(2), cov=Q_z)
                z[1] = wrap_angle(float(z[1]))
                z_list.append((i, z))
        Z.append(z_list)

    # FastSLAM particles
    N = 200
    particles = [Particle(M) for _ in range(N)]
    for p in particles:
        p.x = np.array([0.0, 0.0, 0.0], dtype=float)
        p.w = 1.0 / N

    xs_est = []
    for t in range(T):
        particles = fastslam_1_step(particles, controls[t], Z[t], dt, R_u, Q_z, rng)
        xs_est.append(estimate_pose(particles))

    xs_true = np.array(xs_true[1:], dtype=float)
    xs_est = np.array(xs_est, dtype=float)

    print("Final true pose:", xs_true[-1])
    print("Final est  pose:", xs_est[-1])

    # Extract best particle's landmark map for visualization
    best = max(particles, key=lambda p: p.w)
    est_lms = []
    for i in range(M):
        if best.lms[i].seen:
            est_lms.append(best.lms[i].mu.copy())
        else:
            est_lms.append(np.array([np.nan, np.nan]))
    est_lms = np.array(est_lms, dtype=float)

    if _HAS_PLT:
        plt.figure()
        plt.plot(xs_true[:, 0], xs_true[:, 1], label="true traj")
        plt.plot(xs_est[:, 0], xs_est[:, 1], label="FastSLAM mean traj")
        plt.scatter(landmarks[:, 0], landmarks[:, 1], marker="x", label="true landmarks")
        plt.scatter(est_lms[:, 0], est_lms[:, 1], marker="o", label="best particle landmarks")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.title("FastSLAM 1.0 demo (known data association)")
        plt.show()
    else:
        print("matplotlib not available; skipping plot.")

if __name__ == "__main__":
    main()
