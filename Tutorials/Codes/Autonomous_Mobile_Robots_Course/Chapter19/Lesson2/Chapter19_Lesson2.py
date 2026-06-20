# Chapter19_Lesson2.py
"""
Metrics for Navigation Robustness (Autonomous Mobile Robots)
------------------------------------------------------------
This script computes mission-level robustness metrics from an episode log.

CSV schema (one row per control cycle, sorted by episode_id,time_s):
episode_id,time_s,x,y,goal_x,goal_y,clearance_m,collision,intervention,
goal_reached,tracking_error_m,cmd_v,cmd_v_max,cmd_w,cmd_w_max,recovery_event

Notes:
- collision, intervention, goal_reached, recovery_event are 0/1.
- If no CSV is supplied, the script generates a synthetic dataset and evaluates it.

Robotics library integration notes:
- ROS 2 / Nav2: export odom, cmd_vel, local_costmap clearance, and behavior-tree events
  into this CSV using rosbag2 + a small conversion node/script.
- Python stack: numpy, pandas are used for numerical/statistical processing.
"""

from __future__ import annotations
import sys
from dataclasses import dataclass
from typing import Dict, List, Tuple
import numpy as np
import pandas as pd


@dataclass
class WilsonInterval:
    p_hat: float
    lower: float
    upper: float


def wilson_interval(k: int, n: int, z: float = 1.96) -> WilsonInterval:
    if n <= 0:
        return WilsonInterval(np.nan, np.nan, np.nan)
    p = k / n
    denom = 1.0 + (z * z) / n
    center = (p + (z * z) / (2.0 * n)) / denom
    radius = (z / denom) * np.sqrt((p * (1.0 - p) / n) + (z * z) / (4.0 * n * n))
    return WilsonInterval(p, center - radius, center + radius)


def generate_synthetic_log(num_episodes: int = 20, seed: int = 7) -> pd.DataFrame:
    rng = np.random.default_rng(seed)
    rows = []
    for ep in range(num_episodes):
        T = int(rng.integers(120, 280))
        dt = 0.1
        x, y = 0.0, 0.0
        goal_x, goal_y = 10.0 + rng.normal(0, 0.5), rng.normal(0, 1.0)
        collision_happened = False
        intervention_happened = False
        recovered = False
        for k in range(T):
            t = k * dt
            dxg = goal_x - x
            dyg = goal_y - y
            dist = float(np.hypot(dxg, dyg))
            heading = np.arctan2(dyg, dxg)
            cmd_v_max = 0.8
            cmd_w_max = 1.2
            cmd_v = float(np.clip(0.55 + 0.05 * rng.normal(), -cmd_v_max, cmd_v_max))
            cmd_w = float(np.clip(0.4 * np.sin(0.05 * k) + 0.15 * rng.normal(), -cmd_w_max, cmd_w_max))
            x += cmd_v * dt * np.cos(heading) + 0.01 * rng.normal()
            y += cmd_v * dt * np.sin(heading) + 0.01 * rng.normal()

            # Synthetic clearance with occasional near-obstacle events
            base_clear = 0.9 + 0.25 * np.sin(0.08 * k + ep) + 0.12 * rng.normal()
            if rng.random() < 0.02:
                base_clear -= rng.uniform(0.4, 0.9)
            clearance = max(0.0, base_clear)

            collision = 1 if (clearance < 0.05 and rng.random() < 0.5) else 0
            intervention = 1 if (clearance < 0.12 and rng.random() < 0.2) else 0
            if collision:
                collision_happened = True
            if intervention:
                intervention_happened = True
            recovery_event = 0
            if (collision or intervention) and (not recovered) and rng.random() < 0.7:
                recovery_event = 1
                recovered = True

            tracking_error = abs(0.08 * np.sin(0.03 * k) + 0.04 * rng.normal())
            goal_reached = 1 if dist < 0.35 and not collision_happened else 0

            rows.append(
                [
                    ep, t, x, y, goal_x, goal_y, clearance, collision, intervention,
                    goal_reached, tracking_error, cmd_v, cmd_v_max, cmd_w, cmd_w_max, recovery_event
                ]
            )
            if goal_reached or collision:
                break

    cols = [
        "episode_id", "time_s", "x", "y", "goal_x", "goal_y", "clearance_m", "collision", "intervention",
        "goal_reached", "tracking_error_m", "cmd_v", "cmd_v_max", "cmd_w", "cmd_w_max", "recovery_event"
    ]
    return pd.DataFrame(rows, columns=cols)


def path_length(xs: np.ndarray, ys: np.ndarray) -> float:
    if len(xs) < 2:
        return 0.0
    dx = np.diff(xs)
    dy = np.diff(ys)
    return float(np.sum(np.sqrt(dx * dx + dy * dy)))


def kaplan_meier_failure_survival(episode_df: pd.DataFrame) -> pd.DataFrame:
    """
    Failure event = first collision or intervention in each episode.
    Right-censored if no failure before episode ends.
    """
    records = []
    for ep, g in episode_df.groupby("episode_id"):
        g = g.sort_values("time_s")
        failure_mask = (g["collision"].values > 0) | (g["intervention"].values > 0)
        if np.any(failure_mask):
            idx = int(np.argmax(failure_mask))
            t = float(g.iloc[idx]["time_s"])
            event = 1
        else:
            t = float(g["time_s"].iloc[-1])
            event = 0
        records.append((ep, t, event))
    T = pd.DataFrame(records, columns=["episode_id", "time", "event"]).sort_values("time")

    unique_times = sorted(T.loc[T["event"] == 1, "time"].unique().tolist())
    n = len(T)
    surv = 1.0
    out = []
    for t in unique_times:
        d = int(((T["time"] == t) & (T["event"] == 1)).sum())
        at_risk = int((T["time"] >= t).sum())
        if at_risk > 0:
            surv *= (1.0 - d / at_risk)
        out.append((t, at_risk, d, surv))
    return pd.DataFrame(out, columns=["time_s", "at_risk", "failures", "survival"])


def evaluate_navigation_robustness(df: pd.DataFrame) -> Tuple[pd.DataFrame, Dict[str, float]]:
    required = {
        "episode_id", "time_s", "x", "y", "goal_x", "goal_y", "clearance_m", "collision",
        "intervention", "goal_reached", "tracking_error_m", "cmd_v", "cmd_v_max", "cmd_w", "cmd_w_max", "recovery_event"
    }
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"Missing columns: {sorted(missing)}")

    episode_rows: List[Dict[str, float]] = []
    for ep, g in df.groupby("episode_id"):
        g = g.sort_values("time_s").reset_index(drop=True)
        success = int(g["goal_reached"].max() > 0)
        collision_free = int(g["collision"].sum() == 0)
        intervention_free = int(g["intervention"].sum() == 0)
        had_failure = int((g["collision"].sum() + g["intervention"].sum()) > 0)
        recovered = int((g["recovery_event"].sum() > 0) and success)

        t_final = float(g["time_s"].iloc[-1] - g["time_s"].iloc[0])
        L = path_length(g["x"].to_numpy(), g["y"].to_numpy())
        start = g.iloc[0][["x", "y"]].to_numpy(dtype=float)
        goal = g.iloc[0][["goal_x", "goal_y"]].to_numpy(dtype=float)
        d_ref = float(np.linalg.norm(goal - start))
        path_eff = float(np.clip(d_ref / max(L, 1e-9), 0.0, 1.0)) if success else 0.0

        min_clear = float(g["clearance_m"].min())
        p05_clear = float(np.quantile(g["clearance_m"].to_numpy(), 0.05))
        track_rmse = float(np.sqrt(np.mean(np.square(g["tracking_error_m"].to_numpy()))))

        v_sat = np.abs(g["cmd_v"].to_numpy()) >= 0.98 * np.maximum(g["cmd_v_max"].to_numpy(), 1e-9)
        w_sat = np.abs(g["cmd_w"].to_numpy()) >= 0.98 * np.maximum(g["cmd_w_max"].to_numpy(), 1e-9)
        sat_ratio = float(np.mean(v_sat | w_sat))

        episode_rows.append(
            dict(
                episode_id=int(ep),
                success=success,
                collision_free=collision_free,
                intervention_free=intervention_free,
                had_failure=had_failure,
                recovered_after_failure=recovered,
                completion_time_s=t_final,
                path_length_m=L,
                reference_distance_m=d_ref,
                path_efficiency=path_eff,
                min_clearance_m=min_clear,
                p05_clearance_m=p05_clear,
                tracking_rmse_m=track_rmse,
                saturation_ratio=sat_ratio,
            )
        )

    ep_df = pd.DataFrame(episode_rows)

    n = len(ep_df)
    k_success = int(ep_df["success"].sum())
    k_collision_free = int(ep_df["collision_free"].sum())
    k_intervention_free = int(ep_df["intervention_free"].sum())
    k_recovered = int(ep_df["recovered_after_failure"].sum())
    n_failed = int(ep_df["had_failure"].sum())

    success_ci = wilson_interval(k_success, n)
    collisionfree_ci = wilson_interval(k_collision_free, n)

    # Normalize continuous metrics to [0,1] with task-dependent anchors
    # Anchors should be fixed by the benchmark protocol, not tuned per method.
    tau_time = 60.0  # s
    tau_track = 0.25 # m
    c_safe = 0.30    # m
    time_score = float(np.mean(np.exp(-ep_df["completion_time_s"].to_numpy() / tau_time) * ep_df["success"].to_numpy()))
    track_score = float(np.mean(np.exp(-ep_df["tracking_rmse_m"].to_numpy() / tau_track)))
    clearance_score = float(np.mean(np.clip(ep_df["p05_clearance_m"].to_numpy() / c_safe, 0.0, 1.0)))
    efficiency_score = float(np.mean(ep_df["path_efficiency"].to_numpy()))
    intervention_score = float(k_intervention_free / n)
    collisionfree_score = float(k_collision_free / n)

    weights = {
        "collisionfree": 0.30,
        "intervention_free": 0.15,
        "success": 0.20,
        "clearance": 0.15,
        "efficiency": 0.10,
        "tracking": 0.05,
        "time": 0.05,
    }
    robustness_score = (
        weights["collisionfree"] * collisionfree_score
        + weights["intervention_free"] * intervention_score
        + weights["success"] * (k_success / n)
        + weights["clearance"] * clearance_score
        + weights["efficiency"] * efficiency_score
        + weights["tracking"] * track_score
        + weights["time"] * time_score
    )

    km = kaplan_meier_failure_survival(df)

    summary = {
        "num_episodes": float(n),
        "success_rate": success_ci.p_hat,
        "success_rate_wilson_low": success_ci.lower,
        "success_rate_wilson_high": success_ci.upper,
        "collision_free_rate": collisionfree_ci.p_hat,
        "collision_free_wilson_low": collisionfree_ci.lower,
        "collision_free_wilson_high": collisionfree_ci.upper,
        "intervention_free_rate": float(k_intervention_free / n),
        "mean_completion_time_success_s": float(ep_df.loc[ep_df["success"] == 1, "completion_time_s"].mean()) if k_success > 0 else np.nan,
        "mean_path_efficiency": float(ep_df["path_efficiency"].mean()),
        "mean_p05_clearance_m": float(ep_df["p05_clearance_m"].mean()),
        "mean_tracking_rmse_m": float(ep_df["tracking_rmse_m"].mean()),
        "mean_saturation_ratio": float(ep_df["saturation_ratio"].mean()),
        "recovery_after_failure_rate": float(k_recovered / n_failed) if n_failed > 0 else np.nan,
        "composite_robustness_score": float(robustness_score),
        "km_last_survival": float(km["survival"].iloc[-1]) if len(km) else 1.0,
    }
    return ep_df, summary


def main():
    if len(sys.argv) >= 2:
        df = pd.read_csv(sys.argv[1])
    else:
        df = generate_synthetic_log()
        df.to_csv("Chapter19_Lesson2_synthetic_log.csv", index=False)
        print("Wrote synthetic log to Chapter19_Lesson2_synthetic_log.csv")

    ep_metrics, summary = evaluate_navigation_robustness(df)

    pd.set_option("display.width", 140)
    pd.set_option("display.max_columns", 20)
    print("\nPer-episode metrics (first 10 rows):")
    print(ep_metrics.head(10).to_string(index=False))

    print("\nSummary metrics:")
    for k, v in summary.items():
        print(f"{k:35s} = {v:.6f}" if isinstance(v, (float, np.floating)) else f"{k:35s} = {v}")

    # Save outputs for downstream reporting
    ep_metrics.to_csv("Chapter19_Lesson2_episode_metrics.csv", index=False)
    with open("Chapter19_Lesson2_summary.txt", "w", encoding="utf-8") as f:
        for k, v in summary.items():
            f.write(f"{k},{v}\n")
    print("\nSaved Chapter19_Lesson2_episode_metrics.csv and Chapter19_Lesson2_summary.txt")


if __name__ == "__main__":
    main()
