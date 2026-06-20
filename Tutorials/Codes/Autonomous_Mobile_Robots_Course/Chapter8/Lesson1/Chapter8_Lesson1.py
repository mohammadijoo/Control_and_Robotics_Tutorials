"""
Chapter 8 - Particle-Filter Localization
Lesson 1: Monte Carlo Localization (MCL) Intuition

Minimal from-scratch MCL in a known 2D landmark map.
- State: x = [x, y, theta]
- Control: u = [v, omega] (differential-drive style)
- Measurement: ranges to known landmarks

Dependencies:
  pip install numpy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt


def wrap_angle(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


def motion_model(x: np.ndarray, u: np.ndarray, dt: float,
                 sigma_v: float, sigma_omega: float, rng: np.random.Generator) -> np.ndarray:
    """Sample x_t ~ p(x_t | u_t, x_{t-1})."""
    v = u[0] + rng.normal(0.0, sigma_v)
    w = u[1] + rng.normal(0.0, sigma_omega)

    x_new = x.copy()
    th = x[2]
    if abs(w) < 1e-9:
        x_new[0] += v * dt * np.cos(th)
        x_new[1] += v * dt * np.sin(th)
    else:
        x_new[0] += (v / w) * (np.sin(th + w * dt) - np.sin(th))
        x_new[1] += (v / w) * (-np.cos(th + w * dt) + np.cos(th))
        x_new[2] = wrap_angle(th + w * dt)
    return x_new


def expected_ranges(x: np.ndarray, landmarks: np.ndarray) -> np.ndarray:
    dx = landmarks[:, 0] - x[0]
    dy = landmarks[:, 1] - x[1]
    return np.sqrt(dx * dx + dy * dy)


def gaussian_logpdf(e: np.ndarray, sigma: float) -> float:
    # log N(e; 0, sigma^2 I) up to constant terms included for numerical stability
    return -0.5 * np.sum((e / sigma) ** 2) - e.size * np.log(sigma * np.sqrt(2 * np.pi))


def systematic_resample(particles: np.ndarray, weights: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Systematic resampling: O(N), low variance."""
    n = particles.shape[0]
    positions = (rng.random() + np.arange(n)) / n
    cdf = np.cumsum(weights)
    cdf[-1] = 1.0
    idx = np.searchsorted(cdf, positions)
    return particles[idx].copy()


def mcl_step(particles: np.ndarray, weights: np.ndarray, u: np.ndarray, z: np.ndarray,
             landmarks: np.ndarray, dt: float,
             sigma_v: float, sigma_omega: float, sigma_r: float,
             rng: np.random.Generator):
    # 1) Predict
    for i in range(particles.shape[0]):
        particles[i] = motion_model(particles[i], u, dt, sigma_v, sigma_omega, rng)

    # 2) Update weights via range likelihood p(z | x)
    logw = np.empty(particles.shape[0])
    for i in range(particles.shape[0]):
        zhat = expected_ranges(particles[i], landmarks)
        logw[i] = gaussian_logpdf(z - zhat, sigma_r)
    logw -= np.max(logw)  # stabilize
    weights = np.exp(logw)
    weights /= np.sum(weights)

    # 3) Resample
    particles = systematic_resample(particles, weights, rng)
    weights.fill(1.0 / particles.shape[0])

    # 4) Point estimate (weighted mean after resample is simply average)
    mu = particles.mean(axis=0)
    mu[2] = np.arctan2(np.mean(np.sin(particles[:, 2])), np.mean(np.cos(particles[:, 2])))
    return particles, weights, mu


def simulate_true_robot(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """Noise-free kinematics for 'ground truth' (for demo)."""
    v, w = u
    x_new = x.copy()
    th = x[2]
    if abs(w) < 1e-9:
        x_new[0] += v * dt * np.cos(th)
        x_new[1] += v * dt * np.sin(th)
    else:
        x_new[0] += (v / w) * (np.sin(th + w * dt) - np.sin(th))
        x_new[1] += (v / w) * (-np.cos(th + w * dt) + np.cos(th))
        x_new[2] = wrap_angle(th + w * dt)
    return x_new


def main():
    rng = np.random.default_rng(0)

    # Map: four landmarks at the corners of a square
    landmarks = np.array([[-5.0, -5.0], [5.0, -5.0], [5.0, 5.0], [-5.0, 5.0]])

    # True pose
    x_true = np.array([-3.0, -2.0, 0.3])

    # MCL parameters
    N = 800
    sigma_v = 0.10
    sigma_omega = 0.05
    sigma_r = 0.35
    dt = 0.1

    # Initial belief: broad Gaussian around origin
    particles = np.zeros((N, 3))
    particles[:, 0] = rng.normal(0.0, 3.0, size=N)
    particles[:, 1] = rng.normal(0.0, 3.0, size=N)
    particles[:, 2] = rng.uniform(-np.pi, np.pi, size=N)
    weights = np.full(N, 1.0 / N)

    # Controls: gentle curve
    T = 120
    traj_true = [x_true.copy()]
    traj_est = []

    for t in range(T):
        u = np.array([0.6, 0.25])  # v, omega
        x_true = simulate_true_robot(x_true, u, dt)
        traj_true.append(x_true.copy())

        # Range measurement (noisy)
        z = expected_ranges(x_true, landmarks) + rng.normal(0.0, sigma_r, size=landmarks.shape[0])

        particles, weights, mu = mcl_step(
            particles, weights, u, z, landmarks, dt,
            sigma_v, sigma_omega, sigma_r, rng
        )
        traj_est.append(mu)

    traj_true = np.array(traj_true)
    traj_est = np.array(traj_est)

    # Plot
    plt.figure()
    plt.scatter(particles[:, 0], particles[:, 1], s=5, alpha=0.25, label="particles")
    plt.plot(traj_true[:, 0], traj_true[:, 1], label="true")
    plt.plot(traj_est[:, 0], traj_est[:, 1], label="estimate")
    plt.scatter(landmarks[:, 0], landmarks[:, 1], marker="x", s=80, label="landmarks")
    plt.axis("equal")
    plt.grid(True)
    plt.title("Monte Carlo Localization (MCL) — Intuition Demo")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
