\
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Chapter 8 - Lesson 2: Importance Sampling and Resampling (Particle Filters)

This file is intentionally "from scratch" (NumPy-only) to make the mechanics clear.

Robotics-oriented libraries where these concepts are used:
  - ROS 1/2 navigation stacks (AMCL-style MCL implementations)
  - filterpy (educational), PyParticleEst (academic), Stone Soup (tracking)
  - numpy/scipy for custom research pipelines

Core topics:
  - log-weight normalization (log-sum-exp)
  - Effective Sample Size (ESS)
  - Multinomial, residual, stratified, and systematic resampling
"""

import numpy as np


def log_sum_exp(logw: np.ndarray) -> float:
    """Stable computation of log(sum(exp(logw)))."""
    m = np.max(logw)
    return float(m + np.log(np.sum(np.exp(logw - m))))


def normalize_log_weights(logw: np.ndarray) -> np.ndarray:
    """Return normalized weights from unnormalized log-weights."""
    lse = log_sum_exp(logw)
    w = np.exp(logw - lse)
    # numerical guard: force exact sum=1 (within float)
    w = w / np.sum(w)
    return w


def effective_sample_size(w: np.ndarray) -> float:
    """
    ESS approximation: N_eff = 1 / sum_i w_i^2
    Assumes w are normalized (sum w = 1).
    """
    return float(1.0 / np.sum(np.square(w)))


def multinomial_resample(w: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Multinomial resampling. Returns ancestor indices a[0..N-1]."""
    N = w.size
    return rng.choice(N, size=N, replace=True, p=w)


def systematic_resample(w: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """
    Systematic resampling (low variance, O(N)).
    Returns ancestor indices.
    """
    N = w.size
    positions = (rng.random() + np.arange(N)) / N
    cdf = np.cumsum(w)
    a = np.zeros(N, dtype=int)
    i, j = 0, 0
    while i < N:
        if positions[i] < cdf[j]:
            a[i] = j
            i += 1
        else:
            j += 1
    return a


def stratified_resample(w: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """
    Stratified resampling.
    positions_i ~ U((i-1)/N, i/N), i=1..N
    """
    N = w.size
    positions = (np.arange(N) + rng.random(N)) / N
    cdf = np.cumsum(w)
    a = np.zeros(N, dtype=int)
    i, j = 0, 0
    while i < N:
        if positions[i] < cdf[j]:
            a[i] = j
            i += 1
        else:
            j += 1
    return a


def residual_resample(w: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """
    Residual resampling:
      - deterministically allocate floor(N*w_i) copies
      - resample remaining from residual weights
    """
    N = w.size
    Ns = np.floor(N * w).astype(int)
    R = N - np.sum(Ns)
    a = np.empty(N, dtype=int)
    k = 0
    for i in range(N):
        if Ns[i] > 0:
            a[k:k + Ns[i]] = i
            k += Ns[i]
    if R > 0:
        residual = (N * w - Ns)
        residual = residual / np.sum(residual)
        a[k:] = rng.choice(N, size=R, replace=True, p=residual)
    rng.shuffle(a)
    return a


def demo_1d_tracking(seed: int = 7) -> None:
    """
    Minimal 1D demo mimicking a localization/tracking update:
      x_t = x_{t-1} + u_t + v_t,  v_t ~ N(0, q^2)
      z_t = x_t + n_t,          n_t ~ N(0, r^2)

    We run a few steps, resampling adaptively by ESS.
    """
    rng = np.random.default_rng(seed)

    N = 2000
    q = 0.4  # process std
    r = 0.7  # measurement std

    # initial particles (broad prior)
    x = rng.normal(0.0, 4.0, size=N)
    logw = np.zeros(N)

    x_true = 2.0
    for t in range(1, 9):
        u = 0.3  # commanded motion
        # true system
        x_true = x_true + u + rng.normal(0.0, q)

        # propagate particles (proposal uses motion model)
        x = x + u + rng.normal(0.0, q, size=N)

        # measurement
        z = x_true + rng.normal(0.0, r)

        # importance weights proportional to likelihood p(z|x)
        # log p(z|x) = -0.5*((z-x)^2/r^2) + const
        logw = -0.5 * np.square((z - x) / r)
        w = normalize_log_weights(logw)

        Neff = effective_sample_size(w)
        x_hat = float(np.sum(w * x))
        print(f"t={t:02d}, z={z:+.3f}, true={x_true:+.3f}, est={x_hat:+.3f}, ESS={Neff:.1f}")

        # resample if degeneracy
        if Neff < 0.5 * N:
            a = systematic_resample(w, rng)
            x = x[a]
            logw[:] = 0.0  # reset to uniform
    print("Done.")


if __name__ == "__main__":
    demo_1d_tracking()
