# Chapter17_Lesson3.py
# Autonomous Mobile Robots (Control Engineering) — Chapter 17, Lesson 3
# Next-Best-View (NBV) Strategies for Exploration and Active Mapping
#
# This script provides:
#   1) A minimal occupancy-grid belief representation (probabilities per cell)
#   2) Expected Information Gain (EIG) per candidate viewpoint via a simple sensor model
#   3) A utility function U(v) = EIG(v) - lambda * travel_cost(v)
#   4) A small simulation loop using a hidden "ground-truth" grid world
#
# Dependencies: numpy only (optional matplotlib is NOT used to keep the script minimal)

from __future__ import annotations
import math
import random
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def bernoulli_entropy(p: float, eps: float = 1e-12) -> float:
    """
    Entropy of Bernoulli(p) in nats: H(p) = -p ln p - (1-p) ln (1-p).
    """
    p = clamp(p, eps, 1.0 - eps)
    return -p * math.log(p) - (1.0 - p) * math.log(1.0 - p)


@dataclass
class SensorBinaryModel:
    """
    Binary measurement model z in {occ, free} for each cell (independent approximation).

    Parameters:
      p_hit   = P(z=occ | m=occ)
      p_false = P(z=occ | m=free)   (false positive)
    """
    p_hit: float = 0.85
    p_false: float = 0.15

    def posterior(self, p_occ: float, z_occ: bool) -> float:
        """
        Bayes update for P(m=occ | z).
        """
        p = clamp(p_occ, 1e-12, 1.0 - 1e-12)
        if z_occ:
            num = self.p_hit * p
            den = self.p_hit * p + self.p_false * (1.0 - p)
        else:
            num = (1.0 - self.p_hit) * p
            den = (1.0 - self.p_hit) * p + (1.0 - self.p_false) * (1.0 - p)
        return clamp(num / den, 1e-12, 1.0 - 1e-12)

    def expected_posterior_entropy(self, p_occ: float) -> float:
        """
        E_z[ H(P(m=occ | z)) ] where z ∈ {occ, free}.
        """
        p = clamp(p_occ, 1e-12, 1.0 - 1e-12)
        # P(z=occ) = P(z=occ|m=occ)P(m=occ) + P(z=occ|m=free)P(m=free)
        p_z_occ = self.p_hit * p + self.p_false * (1.0 - p)
        p_z_free = 1.0 - p_z_occ

        p_post_occ = self.posterior(p, True)
        p_post_free = self.posterior(p, False)

        return p_z_occ * bernoulli_entropy(p_post_occ) + p_z_free * bernoulli_entropy(p_post_free)

    def expected_information_gain(self, p_occ: float) -> float:
        """
        IG(p) = H(prior) - E_z[H(posterior)].
        """
        return bernoulli_entropy(p_occ) - self.expected_posterior_entropy(p_occ)


@dataclass
class GridWorld:
    """
    Occupancy grid belief:
      belief[y, x] = P(cell is occupied)
    Ground-truth:
      truth[y, x] ∈ {0,1} (1=occupied)
    """
    width: int
    height: int
    belief: np.ndarray
    truth: np.ndarray

    @staticmethod
    def random_world(width: int, height: int, obstacle_prob: float = 0.18, seed: int = 0) -> "GridWorld":
        rng = np.random.default_rng(seed)
        truth = (rng.random((height, width)) < obstacle_prob).astype(np.int8)
        belief = np.full((height, width), 0.5, dtype=float)  # unknown prior
        return GridWorld(width, height, belief, truth)

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free_truth(self, x: int, y: int) -> bool:
        return self.truth[y, x] == 0

    def entropy_total(self) -> float:
        H = 0.0
        for y in range(self.height):
            for x in range(self.width):
                H += bernoulli_entropy(float(self.belief[y, x]))
        return H


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """
    Bresenham line rasterization from (x0,y0) to (x1,y1), inclusive.
    """
    pts = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        pts.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return pts


def ray_endpoints(cx: int, cy: int, theta: float, r: float) -> Tuple[int, int]:
    ex = int(round(cx + r * math.cos(theta)))
    ey = int(round(cy + r * math.sin(theta)))
    return ex, ey


def expected_ig_view(
    world: GridWorld,
    cx: int,
    cy: int,
    heading: float,
    sensor: SensorBinaryModel,
    fov: float = math.radians(180.0),
    n_rays: int = 45,
    max_range: float = 10.0,
    occ_stop: float = 0.70,
) -> float:
    """
    Expected information gain of a viewpoint, approximated as sum of per-cell IG along rays.
    Occlusion is approximated by stopping a ray once belief occupancy exceeds occ_stop.
    """
    if not world.in_bounds(cx, cy):
        return -1e9

    total = 0.0
    angles = np.linspace(heading - 0.5 * fov, heading + 0.5 * fov, n_rays)
    for ang in angles:
        ex, ey = ray_endpoints(cx, cy, float(ang), max_range)
        line = bresenham(cx, cy, ex, ey)
        # skip first cell (robot position)
        for (x, y) in line[1:]:
            if not world.in_bounds(x, y):
                break
            p_occ = float(world.belief[y, x])
            total += sensor.expected_information_gain(p_occ)

            # occlusion approximation: stop ray if likely-occupied
            if p_occ > occ_stop:
                break
    return total


def travel_cost_grid(cx: int, cy: int, vx: int, vy: int) -> float:
    """
    Simple proxy: Euclidean distance on grid.
    """
    return math.hypot(vx - cx, vy - cy)


def sample_candidate_views(
    world: GridWorld,
    n_candidates: int,
    rng: random.Random,
) -> List[Tuple[int, int, float]]:
    """
    Candidate generation (simplified):
      sample random grid cells that are free in ground truth (sim-only),
      and assign random headings.

    In a real robot, candidates come from:
      - frontier cells (Lesson 1),
      - reachable free-space samples,
      - visibility constraints + collision checking.
    """
    candidates: List[Tuple[int, int, float]] = []
    tries = 0
    while len(candidates) < n_candidates and tries < 50 * n_candidates:
        tries += 1
        x = rng.randrange(world.width)
        y = rng.randrange(world.height)
        if world.is_free_truth(x, y):
            th = rng.random() * 2.0 * math.pi
            candidates.append((x, y, th))
    return candidates


def choose_next_best_view(
    world: GridWorld,
    current: Tuple[int, int, float],
    candidates: List[Tuple[int, int, float]],
    sensor: SensorBinaryModel,
    lam: float = 0.25,
) -> Tuple[int, int, float, float, float]:
    """
    Returns (best_x, best_y, best_theta, best_ig, best_u).
    """
    cx, cy, _ = current
    best = (cx, cy, 0.0, -1e9, -1e9)
    for (vx, vy, th) in candidates:
        ig = expected_ig_view(world, vx, vy, th, sensor)
        cost = travel_cost_grid(cx, cy, vx, vy)
        u = ig - lam * cost
        if u > best[4]:
            best = (vx, vy, th, ig, u)
    return best


def simulate_measurement_and_update(
    world: GridWorld,
    pose: Tuple[int, int, float],
    sensor: SensorBinaryModel,
    fov: float = math.radians(180.0),
    n_rays: int = 60,
    max_range: float = 10.0,
    p_hit_inv: float = 0.70,
    p_false_inv: float = 0.30,
) -> None:
    """
    Simulated scan on ground truth, then Bayesian update of each visited cell with an inverse model.

    Inverse model parameters are distinct from forward model (common practice):
      - If the ray passes through a cell before a hit: treat as 'free' evidence.
      - The first occupied cell on the ray: treat as 'occupied' evidence.
    """
    cx, cy, heading = pose
    angles = np.linspace(heading - 0.5 * fov, heading + 0.5 * fov, n_rays)

    def update_cell(x: int, y: int, z_occ: bool) -> None:
        p = float(world.belief[y, x])
        # Use a simple forward model for the cell update (like SensorBinaryModel but with inverse-tuned params)
        if z_occ:
            p_hit, p_false = p_hit_inv, 1.0 - p_false_inv
        else:
            p_hit, p_false = 1.0 - p_hit_inv, p_false_inv

        # Bayes update:
        num = p_hit * p
        den = p_hit * p + p_false * (1.0 - p)
        world.belief[y, x] = clamp(num / den, 1e-6, 1.0 - 1e-6)

    for ang in angles:
        ex, ey = ray_endpoints(cx, cy, float(ang), max_range)
        line = bresenham(cx, cy, ex, ey)
        for (x, y) in line[1:]:
            if not world.in_bounds(x, y):
                break
            if world.truth[y, x] == 1:
                update_cell(x, y, True)
                break
            else:
                update_cell(x, y, False)


def ascii_map(world: GridWorld, robot_xy: Tuple[int, int]) -> str:
    """
    Simple text view:
      '?' = unknown-ish belief (p around 0.5)
      'o' = believed free (p < 0.35)
      'X' = believed occupied (p > 0.65)
      'R' = robot
    """
    rx, ry = robot_xy
    rows = []
    for y in range(world.height):
        row = []
        for x in range(world.width):
            if (x, y) == (rx, ry):
                row.append("R")
                continue
            p = float(world.belief[y, x])
            if p > 0.65:
                row.append("X")
            elif p < 0.35:
                row.append("o")
            else:
                row.append("?")
        rows.append("".join(row))
    return "\n".join(rows)


def main() -> None:
    rng = random.Random(7)
    world = GridWorld.random_world(width=30, height=18, obstacle_prob=0.16, seed=3)
    sensor = SensorBinaryModel(p_hit=0.85, p_false=0.15)

    # Start pose
    pose = (2, 2, 0.0)
    world.truth[pose[1], pose[0]] = 0  # ensure free in the sim

    print("Initial total entropy (nats):", world.entropy_total())
    for t in range(10):
        candidates = sample_candidate_views(world, n_candidates=80, rng=rng)
        vx, vy, vth, ig, u = choose_next_best_view(world, pose, candidates, sensor, lam=0.22)

        print(f"\nStep {t+1}: NBV = ({vx},{vy}), IG={ig:.2f}, U={u:.2f}")
        pose = (vx, vy, vth)
        simulate_measurement_and_update(world, pose, sensor)

        print("Total entropy (nats):", f"{world.entropy_total():.2f}")
        print(ascii_map(world, (pose[0], pose[1])))

    print("\nDone.")


if __name__ == "__main__":
    main()
