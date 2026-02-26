"""
Chapter6_Lesson2.py
Bayes Filters for Mobile Robots (discrete-state version)

This module implements the Bayes filter recursion for a finite state space:
  bel_t = eta * p(z_t | x_t) * sum_{x_{t-1}} p(x_t | x_{t-1}, u_t) * bel_{t-1}

It also includes a small 1D corridor demo (grid localization) to illustrate
prediction + update steps.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import numpy as np


def normalize(p: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    """Normalize a nonnegative vector to sum to 1."""
    s = float(np.sum(p))
    if s < eps:
        # If everything is (near) zero, fall back to uniform to avoid NaNs.
        return np.ones_like(p) / p.size
    return p / s


@dataclass
class DiscreteBayesFilter:
    """
    Discrete Bayes filter on N states.

    Transition matrix T has shape (N, N) with:
        T[i, j] = P(x_t = i | x_{t-1} = j, u_t)
    """
    N: int

    def predict(self, bel_prev: np.ndarray, T: np.ndarray) -> np.ndarray:
        bel_prev = np.asarray(bel_prev, dtype=float).reshape(self.N)
        T = np.asarray(T, dtype=float).reshape(self.N, self.N)
        # bel_bar[i] = sum_j T[i,j] bel_prev[j]
        bel_bar = T @ bel_prev
        return normalize(bel_bar)

    def update(self, bel_bar: np.ndarray, likelihood: np.ndarray) -> np.ndarray:
        bel_bar = np.asarray(bel_bar, dtype=float).reshape(self.N)
        likelihood = np.asarray(likelihood, dtype=float).reshape(self.N)
        bel = likelihood * bel_bar
        return normalize(bel)

    def step(self, bel_prev: np.ndarray, T: np.ndarray, likelihood: np.ndarray) -> np.ndarray:
        bel_bar = self.predict(bel_prev, T)
        return self.update(bel_bar, likelihood)


def make_shift_transition(N: int, delta: int, p_exact: float = 0.8, p_undershoot: float = 0.1, p_overshoot: float = 0.1) -> np.ndarray:
    """
    A simple probabilistic motion model on a 1D ring (mod N):

    With commanded shift 'delta':
      - exact shift with probability p_exact
      - undershoot (delta-1) with probability p_undershoot
      - overshoot  (delta+1) with probability p_overshoot

    Returns T[i,j] = P(x_t=i | x_{t-1}=j, u_t=delta)
    """
    assert abs((p_exact + p_undershoot + p_overshoot) - 1.0) < 1e-9
    T = np.zeros((N, N), dtype=float)
    for j in range(N):
        i_exact = (j + delta) % N
        i_under = (j + delta - 1) % N
        i_over = (j + delta + 1) % N
        T[i_exact, j] += p_exact
        T[i_under, j] += p_undershoot
        T[i_over, j] += p_overshoot
    return T


def landmark_likelihood(N: int, z: int, landmarks: Iterable[int], p_hit: float = 0.9, p_false: float = 0.1) -> np.ndarray:
    """
    Binary measurement model:
      z = 1 means "sensor detects a landmark"
      z = 0 means "no landmark detected"

    If x in landmarks:
        P(z=1|x)=p_hit, P(z=0|x)=1-p_hit
    else:
        P(z=1|x)=p_false, P(z=0|x)=1-p_false
    """
    L = np.full(N, p_false if z == 1 else (1.0 - p_false), dtype=float)
    landmarks = set(int(k) for k in landmarks)
    for x in landmarks:
        L[x % N] = p_hit if z == 1 else (1.0 - p_hit)
    return L


def demo_1d_corridor(seed: int = 0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Small demo: localization on a 1D ring with 20 cells and two landmarks.

    Returns:
      bel_history: (T+1, N) belief history including initial belief
      truth:       (T+1,)   simulated true states (for inspection)
    """
    rng = np.random.default_rng(seed)
    N = 20
    bf = DiscreteBayesFilter(N=N)

    # Initial belief: uniform
    bel = np.ones(N) / N

    # Landmarks at two positions
    landmarks = [3, 14]

    # Controls and simulated truth
    controls = [1, 1, 1, 2, 1, 1, 2, 1]
    truth = [rng.integers(0, N)]
    zs = []

    # Simulate measurements from truth (with noise)
    for u in controls:
        x_prev = truth[-1]
        # Sample motion (exact/under/over)
        r = rng.random()
        if r < 0.8:
            x = (x_prev + u) % N
        elif r < 0.9:
            x = (x_prev + u - 1) % N
        else:
            x = (x_prev + u + 1) % N
        truth.append(int(x))

        # Sample landmark detection
        at_landmark = (x in landmarks)
        if at_landmark:
            z = 1 if rng.random() < 0.9 else 0
        else:
            z = 1 if rng.random() < 0.1 else 0
        zs.append(int(z))

    bel_hist = [bel.copy()]
    for t, (u, z) in enumerate(zip(controls, zs), start=1):
        Tmat = make_shift_transition(N, delta=u)
        L = landmark_likelihood(N, z=z, landmarks=landmarks)
        bel = bf.step(bel, Tmat, L)
        bel_hist.append(bel.copy())

    return np.vstack(bel_hist), np.array(truth, dtype=int)


if __name__ == "__main__":
    bel_hist, truth = demo_1d_corridor(seed=4)
    print("Final belief (top 5 states):")
    idx = np.argsort(-bel_hist[-1])[:5]
    for i in idx:
        print(f"  state {i:2d}: {bel_hist[-1, i]:.4f}")
    print("Truth trajectory:", truth.tolist())
