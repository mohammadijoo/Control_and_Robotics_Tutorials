# Chapter19_Lesson4.py
# AMR Stress Testing + Paired Ablation (compact reference)

from dataclasses import dataclass
import math
import numpy as np

@dataclass
class Episode:
    stress: float
    success: int
    rmse: float
    margin: float
    path_ratio: float
    ttc: float

WEIGHTS = {"scan": 0.9, "imu": 0.8, "predict": 0.7}

def simulate(seed: int, stress: float, mods: dict) -> Episode:
    rng = np.random.default_rng(seed)
    dyn = rng.poisson(2 + 6 * stress)
    dropout = rng.binomial(1, min(0.85, 0.05 + 0.75 * stress))
    slip = abs(rng.normal(0.0, 0.02 + 0.12 * stress))
    bias = abs(rng.normal(0.0, 0.01 + 0.08 * stress))
    missing = sum(WEIGHTS[k] * (1 - mods[k]) for k in WEIGHTS)

    rmse = 0.05 + 0.09 * stress + 0.04 * dyn + 1.4 * slip + 1.6 * bias + 0.10 * dropout + 0.12 * missing + rng.normal(0, 0.02)
    rmse = max(0.01, rmse)

    margin = 0.55 - 0.18 * stress - 0.01 * dyn - 0.07 * dropout - 0.05 * missing + 0.02 * mods["predict"] + rng.normal(0, 0.03)
    path_ratio = 1.03 + 0.14 * stress + 0.02 * dyn + 0.08 * dropout + 0.05 * missing - 0.03 * mods["scan"] + rng.normal(0, 0.02)
    path_ratio = max(1.0, path_ratio)
    ttc = 34 + 7 * path_ratio + 2.0 * dyn + 8.0 * stress + 3.5 * missing + rng.normal(0, 1.5)

    logit = 3.2 - 2.6 * stress - 2.2 * rmse + 1.0 * margin - 0.5 * (path_ratio - 1) - 0.7 * missing
    p = 1.0 / (1.0 + math.exp(-logit))
    success = int(rng.uniform() < p)
    return Episode(stress, success, rmse, margin, path_ratio, max(5.0, ttc))

def score(ep: Episode) -> float:
    y = 100.0
    y -= 35.0 * (1 - ep.success)
    y -= 16.0 * max(0.0, ep.rmse - 0.10)
    y -= 14.0 * max(0.0, 0.25 - ep.margin)
    y -= 5.0 * max(0.0, ep.path_ratio - 1.10)
    y -= 0.20 * max(0.0, ep.ttc - 45.0)
    return max(0.0, y)

def bootstrap_ci(diffs: np.ndarray, n_boot: int = 2000, alpha: float = 0.05):
    rng = np.random.default_rng(7)
    means = []
    n = len(diffs)
    for _ in range(n_boot):
        idx = rng.integers(0, n, size=n)
        means.append(float(np.mean(diffs[idx])))
    lo = np.quantile(means, alpha / 2)
    hi = np.quantile(means, 1 - alpha / 2)
    return float(lo), float(hi)

def run():
    stress_grid = [0.0, 0.2, 0.4, 0.6, 0.8]
    seeds = list(range(100, 150))

    full = {"scan": 1, "imu": 1, "predict": 1}
    ablations = {
        "Full": full,
        "NoScan": {"scan": 0, "imu": 1, "predict": 1},
        "NoIMU": {"scan": 1, "imu": 0, "predict": 1},
        "NoPredict": {"scan": 1, "imu": 1, "predict": 0},
    }

    records = {}
    for s in stress_grid:
        for name, mods in ablations.items():
            vals = []
            for seed in seeds:
                ep = simulate(seed, s, mods)
                vals.append((seed, ep, score(ep)))
            records[(s, name)] = vals

    print("stress, ablation, success_rate, mean_score, mean_rmse")
    for s in stress_grid:
        for name in ablations:
            vals = records[(s, name)]
            succ = np.mean([v[1].success for v in vals])
            mean_score = np.mean([v[2] for v in vals])
            mean_rmse = np.mean([v[1].rmse for v in vals])
            print(f"{s:.1f}, {name:9s}, {succ:.3f}, {mean_score:.2f}, {mean_rmse:.3f}")

    print("\nPaired deltas (Full - Ablated) with bootstrap CI")
    for s in stress_grid:
        full_by_seed = {seed: sc for seed, _, sc in records[(s, "Full")]}
        for name in ["NoScan", "NoIMU", "NoPredict"]:
            diffs = np.array([full_by_seed[seed] - sc for seed, _, sc in records[(s, name)]], dtype=float)
            lo, hi = bootstrap_ci(diffs)
            print(f"stress={s:.1f}, {name:9s}, delta={diffs.mean():6.2f}, CI95=({lo:6.2f},{hi:6.2f})")

if __name__ == "__main__":
    run()
