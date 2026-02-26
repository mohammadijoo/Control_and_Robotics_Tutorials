#!/usr/bin/env python3
# Chapter9_Lesson4.py
# Elevation + traversability maps (outdoor AMR) — minimal, from-scratch implementation.

import math
import numpy as np
import matplotlib.pyplot as plt


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


class ElevationTraversabilityMap:
    """
    Grid-based elevation map where each cell stores:
      - mu: posterior mean of terrain elevation (meters)
      - sigma2: posterior variance (m^2)

    Updates are done with a 1D Kalman filter per cell:
      z = h + v,  v ~ N(0, R)
    """

    def __init__(self, x_min, x_max, y_min, y_max, resolution,
                 sigma0=2.0,  # initial std dev in meters
                 meas_sigma=0.10,  # sensor std dev in meters
                 gate_k=3.0):
        self.x_min = float(x_min)
        self.x_max = float(x_max)
        self.y_min = float(y_min)
        self.y_max = float(y_max)
        self.res = float(resolution)

        self.nx = int(math.ceil((self.x_max - self.x_min) / self.res))
        self.ny = int(math.ceil((self.y_max - self.y_min) / self.res))

        # Mean and variance fields
        self.mu = np.zeros((self.nx, self.ny), dtype=np.float64)
        self.sigma2 = (sigma0 ** 2) * np.ones((self.nx, self.ny), dtype=np.float64)

        # Simple bookkeeping: number of fusions per cell
        self.count = np.zeros((self.nx, self.ny), dtype=np.int32)

        self.R = float(meas_sigma) ** 2
        self.gate_k = float(gate_k)

    def _to_index(self, x, y):
        ix = int((x - self.x_min) / self.res)
        iy = int((y - self.y_min) / self.res)
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            return ix, iy
        return None

    def update_point(self, x, y, z):
        idx = self._to_index(x, y)
        if idx is None:
            return False
        ix, iy = idx

        mu = self.mu[ix, iy]
        s2 = self.sigma2[ix, iy]

        # Innovation
        nu = z - mu
        S = s2 + self.R

        # Robust gate: if far from current belief, reject as obstacle/outlier
        if self.count[ix, iy] > 0 and abs(nu) > self.gate_k * math.sqrt(S):
            return False

        # Kalman gain
        K = s2 / S

        # Posterior
        self.mu[ix, iy] = mu + K * nu
        self.sigma2[ix, iy] = (1.0 - K) * s2
        self.count[ix, iy] += 1
        return True

    def update_point_cloud(self, pts_xyz):
        accepted = 0
        for x, y, z in pts_xyz:
            accepted += int(self.update_point(float(x), float(y), float(z)))
        return accepted

    def compute_features(self):
        """
        Compute local geometric features from elevation:
          - slope: arctan(||grad h||) (radians)
          - roughness: std-dev of neighborhood heights (meters)
          - step: max neighbor height difference (meters)
        """
        mu = self.mu
        nx, ny = mu.shape

        slope = np.zeros_like(mu)
        rough = np.zeros_like(mu)
        step = np.zeros_like(mu)

        # Use finite-difference gradients (skip borders)
        for i in range(1, nx - 1):
            for j in range(1, ny - 1):
                if self.count[i, j] == 0:
                    continue

                dzdx = (mu[i + 1, j] - mu[i - 1, j]) / (2.0 * self.res)
                dzdy = (mu[i, j + 1] - mu[i, j - 1]) / (2.0 * self.res)
                slope[i, j] = math.atan(math.sqrt(dzdx * dzdx + dzdy * dzdy))

                neigh = mu[i - 1:i + 2, j - 1:j + 2].reshape(-1)
                rough[i, j] = float(np.std(neigh))

                # Step: max absolute neighbor difference
                diffs = np.abs(neigh - mu[i, j])
                step[i, j] = float(np.max(diffs))

        return slope, rough, step

    def traversability_cost(self, slope, rough, step,
                            slope_ref=0.35,   # ~20 deg in rad
                            rough_ref=0.08,   # meters
                            step_ref=0.12,    # meters
                            w_s=3.0, w_r=2.0, w_d=2.5,
                            bias=-3.0):
        """
        Map (slope, roughness, step) -> [0,1] cost where 0 is easy and 1 is hard.
        """
        # Normalize features
        s = slope / max(1e-9, float(slope_ref))
        r = rough / max(1e-9, float(rough_ref))
        d = step / max(1e-9, float(step_ref))

        score = w_s * s + w_r * r + w_d * d + bias
        return sigmoid(score)

    def save_csv(self, path_prefix):
        np.savetxt(path_prefix + "_elevation_mu.csv", self.mu, delimiter=",")
        np.savetxt(path_prefix + "_elevation_sigma2.csv", self.sigma2, delimiter=",")
        np.savetxt(path_prefix + "_count.csv", self.count, delimiter=",")


def generate_synthetic_point_cloud(n_ground=50000, n_outliers=2000, seed=7):
    """
    Synthetic terrain z_true(x,y) + noisy measurements, plus some obstacle outliers.
    """
    rng = np.random.default_rng(seed)

    # Terrain domain
    x = rng.uniform(-10.0, 10.0, size=n_ground)
    y = rng.uniform(-10.0, 10.0, size=n_ground)

    # Smooth terrain
    z_true = (
        0.20 * np.sin(0.35 * x) +
        0.15 * np.cos(0.25 * y) +
        0.03 * x
    )

    # Add a hill/bump (still traversable)
    z_true += 0.25 * np.exp(-0.08 * ((x - 2.0) ** 2 + (y + 1.0) ** 2))

    # Sensor noise
    z_meas = z_true + rng.normal(0.0, 0.10, size=n_ground)

    ground = np.stack([x, y, z_meas], axis=1)

    # Outliers (e.g., vegetation/rocks) that should be rejected by gating
    xo = rng.uniform(-10.0, 10.0, size=n_outliers)
    yo = rng.uniform(-10.0, 10.0, size=n_outliers)
    zo = (
        0.20 * np.sin(0.35 * xo) +
        0.15 * np.cos(0.25 * yo) +
        0.03 * xo +
        0.8 + rng.normal(0.0, 0.05, size=n_outliers)
    )
    outliers = np.stack([xo, yo, zo], axis=1)

    pts = np.concatenate([ground, outliers], axis=0)
    rng.shuffle(pts, axis=0)
    return pts


def main():
    # Build map
    emap = ElevationTraversabilityMap(
        x_min=-10.0, x_max=10.0, y_min=-10.0, y_max=10.0,
        resolution=0.20,
        sigma0=2.0, meas_sigma=0.10, gate_k=3.0
    )

    pts = generate_synthetic_point_cloud()
    accepted = emap.update_point_cloud(pts)
    print("Points accepted:", accepted, "out of", pts.shape[0])

    slope, rough, step = emap.compute_features()
    cost = emap.traversability_cost(slope, rough, step)

    # Visualize
    plt.figure()
    plt.title("Elevation mean (m)")
    plt.imshow(emap.mu.T, origin="lower", aspect="equal")
    plt.colorbar()

    plt.figure()
    plt.title("Traversability cost (0 easy, 1 hard)")
    plt.imshow(cost.T, origin="lower", aspect="equal")
    plt.colorbar()

    plt.show()

    # Export
    emap.save_csv("Chapter9_Lesson4_output")


if __name__ == "__main__":
    main()
