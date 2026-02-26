# Chapter8_Lesson4.py
# Particle-Filter Localization — Degeneracy + Kidnapped Robot Recovery (2D)
#
# Dependencies: numpy, matplotlib (optional)
#
# Run:
#   python Chapter8_Lesson4.py
#
# This script simulates a 2D robot with range-bearing measurements to known landmarks,
# runs Monte Carlo Localization (MCL), monitors degeneracy via N_eff, and
# recovers from a "kidnapped robot" event by adaptive random particle injection.

import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_PLOT = True
except Exception:
    HAS_PLOT = False


def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Pose:
    x: float
    y: float
    th: float


@dataclass
class Particle:
    x: float
    y: float
    th: float
    w: float


def motion_step(p: Particle, v: float, w: float, dt: float,
                sigma_v: float, sigma_w: float, rng: np.random.Generator) -> Particle:
    """Unicycle motion with additive Gaussian noise on controls."""
    v_n = v + rng.normal(0.0, sigma_v)
    w_n = w + rng.normal(0.0, sigma_w)

    if abs(w_n) < 1e-9:
        x2 = p.x + v_n * dt * math.cos(p.th)
        y2 = p.y + v_n * dt * math.sin(p.th)
        th2 = p.th
    else:
        x2 = p.x + (v_n / w_n) * (math.sin(p.th + w_n * dt) - math.sin(p.th))
        y2 = p.y - (v_n / w_n) * (math.cos(p.th + w_n * dt) - math.cos(p.th))
        th2 = p.th + w_n * dt

    th2 = wrap_angle(th2)
    return Particle(x2, y2, th2, p.w)


def measurement_model(pose: Pose, landmarks: np.ndarray) -> np.ndarray:
    """
    Deterministic range-bearing (r, b) to each landmark.
    Returns array shape (M,2): [range, bearing].
    """
    dx = landmarks[:, 0] - pose.x
    dy = landmarks[:, 1] - pose.y
    r = np.sqrt(dx * dx + dy * dy)
    b = np.arctan2(dy, dx) - pose.th
    b = (b + np.pi) % (2.0 * np.pi) - np.pi
    return np.stack([r, b], axis=1)


def log_sensor_likelihood(p: Particle, z: np.ndarray, landmarks: np.ndarray,
                          sigma_r: float, sigma_b: float) -> float:
    """Log-likelihood log p(z|x) with Gaussian independent errors."""
    zh = measurement_model(Pose(p.x, p.y, p.th), landmarks)
    dr = z[:, 0] - zh[:, 0]
    db = z[:, 1] - zh[:, 1]
    db = (db + np.pi) % (2.0 * np.pi) - np.pi

    c_r = -math.log(math.sqrt(2.0 * math.pi) * sigma_r)
    c_b = -math.log(math.sqrt(2.0 * math.pi) * sigma_b)
    ll = 0.0
    for i in range(z.shape[0]):
        ll += -0.5 * float(dr[i] / sigma_r) ** 2 + c_r
        ll += -0.5 * float(db[i] / sigma_b) ** 2 + c_b
    return ll


def normalize_weights(particles: List[Particle]) -> None:
    s = sum(p.w for p in particles)
    if s <= 0.0 or not np.isfinite(s):
        n = len(particles)
        for p in particles:
            p.w = 1.0 / n
        return
    for p in particles:
        p.w /= s


def effective_sample_size(particles: List[Particle]) -> float:
    """N_eff = 1 / sum_i w_i^2 (assuming normalized weights)."""
    s2 = sum((p.w * p.w) for p in particles)
    if s2 <= 0.0:
        return 0.0
    return 1.0 / s2


def systematic_resample(particles: List[Particle], rng: np.random.Generator) -> List[Particle]:
    """Systematic resampling. Returns equally weighted particles."""
    n = len(particles)
    weights = np.array([p.w for p in particles], dtype=float)
    cdf = np.cumsum(weights)
    cdf[-1] = 1.0

    u0 = rng.random() / n
    js = []
    i = 0
    for m in range(n):
        u = u0 + m / n
        while u > cdf[i]:
            i += 1
        js.append(i)

    return [Particle(particles[j].x, particles[j].y, particles[j].th, 1.0 / n) for j in js]


def roughen(particles: List[Particle], k: float, bounds: Tuple[float, float, float, float],
            rng: np.random.Generator) -> None:
    """Post-resampling jitter (roughening) to fight impoverishment."""
    xmin, xmax, ymin, ymax = bounds
    n = len(particles)
    d = 3.0
    sx = k * (xmax - xmin) * (n ** (-1.0 / d))
    sy = k * (ymax - ymin) * (n ** (-1.0 / d))
    sth = k * (2.0 * math.pi) * (n ** (-1.0 / d))

    for p in particles:
        p.x += rng.normal(0.0, sx)
        p.y += rng.normal(0.0, sy)
        p.th = wrap_angle(p.th + rng.normal(0.0, sth))


def inject_random_particles(particles: List[Particle], frac: float,
                            bounds: Tuple[float, float, float, float],
                            rng: np.random.Generator) -> None:
    """Replace a fraction of particles with global random samples."""
    n = len(particles)
    m = int(max(0, min(n, round(frac * n))))
    if m == 0:
        return
    xmin, xmax, ymin, ymax = bounds
    idx = rng.choice(n, size=m, replace=False)
    for j in idx:
        particles[j].x = float(rng.uniform(xmin, xmax))
        particles[j].y = float(rng.uniform(ymin, ymax))
        particles[j].th = float(rng.uniform(-math.pi, math.pi))


def estimate_pose(particles: List[Particle]) -> Pose:
    """Weighted mean for x,y and circular mean for theta."""
    ws = np.array([p.w for p in particles], dtype=float)
    xs = np.array([p.x for p in particles], dtype=float)
    ys = np.array([p.y for p in particles], dtype=float)
    ths = np.array([p.th for p in particles], dtype=float)

    x = float(np.sum(ws * xs))
    y = float(np.sum(ws * ys))
    c = float(np.sum(ws * np.cos(ths)))
    s = float(np.sum(ws * np.sin(ths)))
    th = math.atan2(s, c)
    return Pose(x, y, th)


def run_demo():
    rng = np.random.default_rng(4)

    bounds = (0.0, 10.0, 0.0, 10.0)
    landmarks = np.array([[2.0, 2.0],
                          [8.0, 2.0],
                          [8.0, 8.0],
                          [2.0, 8.0]], dtype=float)

    dt = 0.1
    T = 300
    kidnapped_t = 170

    sigma_v = 0.05
    sigma_w = 0.03

    sigma_r = 0.15
    sigma_b = 0.07

    N = 800
    N_eff_ratio = 0.5
    roughen_k = 0.15

    eps_min = 0.01
    eps_max = 0.30
    ll_thresh = -12.0

    x_true = Pose(1.0, 1.0, 0.0)

    particles = [
        Particle(float(rng.uniform(bounds[0], bounds[1])),
                 float(rng.uniform(bounds[2], bounds[3])),
                 float(rng.uniform(-math.pi, math.pi)),
                 1.0 / N)
        for _ in range(N)
    ]

    true_hist = []
    est_hist = []
    Neff_hist = []
    eps_hist = []

    def control(t: int):
        if t < 70:
            return 0.7, 0.0
        if t < 90:
            return 0.7, 0.9
        if t < 160:
            return 0.7, 0.0
        if t < 180:
            return 0.7, 0.9
        if t < 250:
            return 0.7, 0.0
        return 0.7, 0.9

    for t in range(T):
        v_cmd, w_cmd = control(t)

        # True motion
        x_true_p = Particle(x_true.x, x_true.y, x_true.th, 1.0)
        x_true_p = motion_step(x_true_p, v_cmd, w_cmd, dt, sigma_v, sigma_w, rng)
        x_true = Pose(x_true_p.x, x_true_p.y, x_true_p.th)

        if t == kidnapped_t:
            x_true = Pose(float(rng.uniform(bounds[0], bounds[1])),
                          float(rng.uniform(bounds[2], bounds[3])),
                          float(rng.uniform(-math.pi, math.pi)))

        z = measurement_model(x_true, landmarks)
        z[:, 0] += rng.normal(0.0, sigma_r, size=z.shape[0])
        z[:, 1] += rng.normal(0.0, sigma_b, size=z.shape[0])
        z[:, 1] = (z[:, 1] + np.pi) % (2.0 * np.pi) - np.pi

        particles = [motion_step(p, v_cmd, w_cmd, dt, sigma_v, sigma_w, rng) for p in particles]

        # Weight update (log domain)
        lls = np.array([log_sensor_likelihood(p, z, landmarks, sigma_r, sigma_b) for p in particles], dtype=float)
        lls = lls - np.max(lls)
        ws = np.exp(lls)
        for i, p in enumerate(particles):
            p.w = float(ws[i])
        normalize_weights(particles)

        # Mismatch score: weighted mean log-likelihood
        lbar = float(np.sum(np.array([p.w for p in particles]) *
                            np.array([log_sensor_likelihood(p, z, landmarks, sigma_r, sigma_b) for p in particles])))
        eps = eps_max if (lbar < ll_thresh) else eps_min

        Neff = effective_sample_size(particles)

        if Neff < N_eff_ratio * N:
            particles = systematic_resample(particles, rng)
            roughen(particles, k=roughen_k, bounds=bounds, rng=rng)
            inject_random_particles(particles, frac=eps, bounds=bounds, rng=rng)

            # Re-weight after injection
            lls2 = np.array([log_sensor_likelihood(p, z, landmarks, sigma_r, sigma_b) for p in particles], dtype=float)
            lls2 = lls2 - np.max(lls2)
            ws2 = np.exp(lls2)
            for i, p in enumerate(particles):
                p.w = float(ws2[i])
            normalize_weights(particles)

        x_est = estimate_pose(particles)

        true_hist.append((x_true.x, x_true.y, x_true.th))
        est_hist.append((x_est.x, x_est.y, x_est.th))
        Neff_hist.append(Neff)
        eps_hist.append(eps)

    true_hist = np.array(true_hist)
    est_hist = np.array(est_hist)

    out_csv = "Chapter8_Lesson4_results.csv"
    with open(out_csv, "w", encoding="utf-8") as f:
        f.write("t,true_x,true_y,true_th,est_x,est_y,est_th,Neff,eps\n")
        for t in range(T):
            f.write(f"{t},{true_hist[t,0]},{true_hist[t,1]},{true_hist[t,2]},"
                    f"{est_hist[t,0]},{est_hist[t,1]},{est_hist[t,2]},"
                    f"{Neff_hist[t]},{eps_hist[t]}\n")
    print("Saved:", out_csv)

    if HAS_PLOT:
        plt.figure()
        plt.plot(true_hist[:, 0], true_hist[:, 1], label="true")
        plt.plot(est_hist[:, 0], est_hist[:, 1], label="estimate")
        plt.scatter(landmarks[:, 0], landmarks[:, 1], marker="x", label="landmarks")
        plt.axis("equal")
        plt.legend()
        plt.title(f"MCL trajectory (kidnapped at t={kidnapped_t})")

        plt.figure()
        plt.plot(Neff_hist)
        plt.axhline(y=N_eff_ratio * N, linestyle="--")
        plt.title("Effective sample size N_eff")

        plt.figure()
        plt.plot(eps_hist)
        plt.title("Injection fraction eps")
        plt.show()
    else:
        print("matplotlib not available; skipping plots.")


if __name__ == "__main__":
    run_demo()
