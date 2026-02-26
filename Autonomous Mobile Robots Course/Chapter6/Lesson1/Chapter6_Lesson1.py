"""
Chapter 6 - Lesson 1: Belief as a Probability Distribution Over Pose
Autonomous Mobile Robots (Control Engineering)

This script demonstrates how to represent a belief over planar pose x = (x, y, theta)
as (i) a Gaussian approximation, (ii) a discrete grid approximation of a PDF, and
(iii) a particle set sampled from the grid.

Focus: normalization, expectations, and circular statistics for heading.
"""

from __future__ import annotations
import numpy as np
from dataclasses import dataclass


def wrap_to_pi(angle: float | np.ndarray) -> float | np.ndarray:
    """Wrap angle(s) to (-pi, pi]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class GaussianBeliefSE2:
    """A simple Gaussian belief over R^2 x S^1 (theta approximated on R with wrapping)."""
    mu: np.ndarray      # shape (3,)
    Sigma: np.ndarray   # shape (3,3)

    def pdf(self, x: np.ndarray) -> float:
        x = np.asarray(x, dtype=float).reshape(3,)
        dx = x - self.mu
        dx[2] = wrap_to_pi(dx[2])
        invS = np.linalg.inv(self.Sigma)
        detS = np.linalg.det(self.Sigma)
        norm = 1.0 / np.sqrt(((2.0 * np.pi) ** 3) * detS)
        return float(norm * np.exp(-0.5 * dx.T @ invS @ dx))


def grid_belief_from_gaussian(mu, Sigma, xs, ys, thetas):
    """Discretize a Gaussian belief onto a grid and normalize w.r.t. cell volume."""
    gb = np.zeros((len(xs), len(ys), len(thetas)), dtype=float)
    g = GaussianBeliefSE2(mu=np.array(mu, dtype=float), Sigma=np.array(Sigma, dtype=float))

    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            for k, th in enumerate(thetas):
                gb[i, j, k] = g.pdf(np.array([x, y, th], dtype=float))

    dx = float(xs[1] - xs[0])
    dy = float(ys[1] - ys[0])
    dth = float(thetas[1] - thetas[0])
    cell_vol = dx * dy * dth

    Z = gb.sum() * cell_vol
    gb /= Z
    return gb, cell_vol


def grid_expectation(xs, ys, thetas, b, cell_vol):
    """Compute E[x], E[y], and circular mean of theta for a grid belief."""
    X, Y, TH = np.meshgrid(xs, ys, thetas, indexing="ij")

    ex = np.sum(X * b) * cell_vol
    ey = np.sum(Y * b) * cell_vol

    esin = np.sum(np.sin(TH) * b) * cell_vol
    ecos = np.sum(np.cos(TH) * b) * cell_vol
    eth = np.arctan2(esin, ecos)
    return np.array([ex, ey, eth], dtype=float)


def main():
    np.random.seed(7)

    # Gaussian belief parameters
    mu = np.array([2.0, -1.0, 0.7], dtype=float)
    Sigma = np.diag([0.2**2, 0.3**2, (10.0 * np.pi / 180.0) ** 2])

    belief = GaussianBeliefSE2(mu=mu, Sigma=Sigma)

    x_test = np.array([2.1, -1.2, 0.75], dtype=float)
    print("pdf(x_test) =", belief.pdf(x_test))

    # Grid discretization (R^2 x S^1)
    xs = np.linspace(1.0, 3.0, 101)
    ys = np.linspace(-2.0, 0.0, 101)
    thetas = np.linspace(-np.pi, np.pi, 121, endpoint=False)

    b, cell_vol = grid_belief_from_gaussian(mu, Sigma, xs, ys, thetas)
    print("grid normalization check:", b.sum() * cell_vol)

    mu_hat = grid_expectation(xs, ys, thetas, b, cell_vol)
    print("grid E[pose] =", mu_hat)

    # Sample particles from the grid belief
    N = 2000
    probs = (b * cell_vol).ravel()
    probs /= probs.sum()

    idx = np.random.choice(probs.size, size=N, replace=True, p=probs)
    ix, iy, ith = np.unravel_index(idx, b.shape)
    particles = np.vstack([xs[ix], ys[iy], thetas[ith]]).T

    mean_xy = particles[:, :2].mean(axis=0)
    mean_th = np.arctan2(np.sin(particles[:, 2]).mean(), np.cos(particles[:, 2]).mean())
    print("particle mean approx =", np.array([mean_xy[0], mean_xy[1], mean_th]))


if __name__ == "__main__":
    main()
