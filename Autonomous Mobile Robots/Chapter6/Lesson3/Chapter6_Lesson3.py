# Chapter6_Lesson3.py
# Autonomous Mobile Robots (Control Engineering) - Chapter 6, Lesson 3
# Motion Models vs Sensor Models: A discrete Bayes-filter demonstration (1D grid)

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List

import numpy as np


def gaussian_pdf(x: np.ndarray, mu: float, sigma: float) -> np.ndarray:
    # Univariate Gaussian density N(x; mu, sigma^2) evaluated elementwise.
    sigma = float(sigma)
    if sigma <= 0.0:
        raise ValueError("sigma must be > 0")
    return (1.0 / (math.sqrt(2.0 * math.pi) * sigma)) * np.exp(-0.5 * ((x - mu) / sigma) ** 2)


def normalize(p: np.ndarray, eps: float = 1e-15) -> np.ndarray:
    s = float(np.sum(p))
    if s < eps:
        raise ValueError("Probability mass nearly zero; check likelihood / parameters.")
    return p / s


@dataclass
class OneDWorld:
    # 1D corridor world discretized into N cells of size dx (meters).
    N: int = 101
    dx: float = 0.1
    beacon_x: float = 4.0  # beacon position in meters

    @property
    def xs(self) -> np.ndarray:
        return np.arange(self.N) * self.dx


def motion_predict(bel: np.ndarray, u: float, sigma_u: float, world: OneDWorld) -> np.ndarray:
    # Motion model: x_t = x_{t-1} + u + w, w ~ N(0, sigma_u^2).
    # Discrete prediction: bel_bar[x] = sum_{x'} p(x | u, x') bel[x'].
    xs = world.xs
    bel_bar = np.zeros_like(bel)

    # For each previous state x', spread its mass according to Gaussian over x.
    for i, x_prev in enumerate(xs):
        mu = x_prev + u
        kernel = gaussian_pdf(xs, mu, sigma_u)
        bel_bar += bel[i] * kernel

    return normalize(bel_bar)


def sensor_update(bel_bar: np.ndarray, z: float, sigma_z: float, world: OneDWorld) -> np.ndarray:
    # Sensor model: z = |x - beacon_x| + v, v ~ N(0, sigma_z^2).
    # Measurement likelihood: p(z | x) = N(z; |x - beacon_x|, sigma_z^2).
    xs = world.xs
    expected = np.abs(xs - world.beacon_x)
    likelihood = gaussian_pdf(np.array([z], dtype=float), expected, sigma_z).reshape(-1)
    post = bel_bar * likelihood
    return normalize(post)


def run_demo() -> None:
    world = OneDWorld(N=121, dx=0.1, beacon_x=6.0)

    # Initial belief: uniform
    bel = np.ones(world.N, dtype=float)
    bel = normalize(bel)

    # Controls and measurements (synthetic)
    u_seq = [0.5, 0.5, 0.5, 0.5]  # move 0.5 m each step
    z_seq = [5.5, 5.0, 4.5, 4.0]  # observed distance-to-beacon decreasing

    sigma_u = 0.25  # motion uncertainty (meters)
    sigma_z = 0.35  # sensor uncertainty (meters)

    print("t |  MAP estimate (m) |  belief entropy (nats)")
    print("--+-------------------+----------------------")

    for t, (u, z) in enumerate(zip(u_seq, z_seq), start=1):
        bel_bar = motion_predict(bel, u=u, sigma_u=sigma_u, world=world)
        bel = sensor_update(bel_bar, z=z, sigma_z=sigma_z, world=world)

        x_map = world.xs[int(np.argmax(bel))]
        entropy = -float(np.sum(np.where(bel > 0, bel * np.log(bel), 0.0)))
        print(f"{t:>1} | {x_map:>17.3f} | {entropy:>20.6f}")

    # Export belief to CSV (useful for plotting elsewhere)
    out = np.column_stack([world.xs, bel])
    np.savetxt("Chapter6_Lesson3_belief_final.csv", out, delimiter=",", header="x,bel", comments="")
    print("\nSaved: Chapter6_Lesson3_belief_final.csv")


if __name__ == "__main__":
    run_demo()
