#!/usr/bin/env python3
# Chapter6_Lesson5.py
# Lab: Build a Basic Bayes Filter Loop (discrete 1D cyclic state)

import math
import numpy as np

def wrap_interval(x, half_period):
    """Wrap x into [-half_period, +half_period)."""
    period = 2.0 * half_period
    return (x + half_period) % period - half_period

def gaussian_pdf(x, mu, sigma):
    return (1.0 / (math.sqrt(2.0 * math.pi) * sigma)) * math.exp(-0.5 * ((x - mu) / sigma) ** 2)

def build_cyclic_motion_kernel(N, dx, delta_m, sigma_m):
    """Return kernel k over cyclic index offsets so that prediction is cyclic convolution."""
    mu_cells = delta_m / dx
    sigma_cells = sigma_m / dx

    # offsets in [-N/2, N/2)
    k = np.zeros(N, dtype=float)
    half = N // 2
    for idx in range(N):
        d = idx
        if idx >= half:
            d = idx - N  # negative offset
        k[idx] = math.exp(-0.5 * ((d - mu_cells) / sigma_cells) ** 2)

    k /= np.sum(k)
    return k

def predict_cyclic_fft(bel_prev, kernel):
    """Cyclic convolution using FFT: bel_bar = bel_prev (*) kernel."""
    B = np.fft.fft(bel_prev)
    K = np.fft.fft(kernel)
    bel_bar = np.real(np.fft.ifft(B * K))
    bel_bar = np.maximum(bel_bar, 0.0)
    bel_bar /= np.sum(bel_bar)
    return bel_bar

def likelihood_vector(N, dx, landmark_x, z_meas, sigma_z, half_period):
    xs = np.arange(N) * dx
    # signed displacement to landmark in [-L/2, L/2)
    d = np.array([wrap_interval(x - landmark_x, half_period) for x in xs])
    l = np.array([gaussian_pdf(z_meas, di, sigma_z) for di in d])
    l = np.maximum(l, 1e-300)
    return l

def bayes_filter_step(bel_prev, u_delta_m, kernel, z_meas, sigma_z, landmark_x, dx, half_period):
    bel_bar = predict_cyclic_fft(bel_prev, kernel)
    l = likelihood_vector(len(bel_prev), dx, landmark_x, z_meas, sigma_z, half_period)
    bel = bel_bar * l
    bel /= np.sum(bel)
    return bel, bel_bar

def run_demo():
    # Discretization
    L = 10.0       # [m] track length (cyclic)
    N = 200        # number of grid cells
    dx = L / N
    half_period = L / 2.0

    # Models
    u_delta_m = 0.35      # commanded translation per step [m]
    sigma_m   = 0.12      # motion noise std [m]
    sigma_z   = 0.20      # sensor noise std [m]
    landmark_x = 2.0      # landmark position [m] on the ring

    # Build motion kernel once (time-invariant for fixed u)
    kernel = build_cyclic_motion_kernel(N, dx, u_delta_m, sigma_m)

    # Initial belief: uniform
    bel = np.ones(N, dtype=float) / N

    # Ground truth (for simulation only)
    rng = np.random.default_rng(7)
    x_true = 7.0

    T = 25
    print("t, x_true[m], x_hat_MAP[m], z_meas[m]")
    for t in range(1, T + 1):
        # Simulate motion
        x_true = (x_true + u_delta_m + rng.normal(0.0, sigma_m)) % L

        # Simulate sensor: signed displacement to landmark
        z_true = wrap_interval(x_true - landmark_x, half_period)
        z_meas = wrap_interval(z_true + rng.normal(0.0, sigma_z), half_period)

        # Bayes filter update
        bel, bel_bar = bayes_filter_step(
            bel_prev=bel,
            u_delta_m=u_delta_m,
            kernel=kernel,
            z_meas=z_meas,
            sigma_z=sigma_z,
            landmark_x=landmark_x,
            dx=dx,
            half_period=half_period
        )

        idx_hat = int(np.argmax(bel))
        x_hat = idx_hat * dx
        print(f"{t:2d}, {x_true:7.3f}, {x_hat:10.3f}, {z_meas:7.3f}")

    # Optional: report posterior entropy as a sanity check
    eps = 1e-12
    H = -np.sum(bel * np.log(bel + eps))
    print(f"Posterior entropy (nats): {H:.4f}")

if __name__ == "__main__":
    run_demo()
