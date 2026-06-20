# Chapter19_Lesson3.py
# Standard Datasets and Simulated Benchmarks for AMR
# Python reference implementation (NumPy/Pandas/Matplotlib)
#
# This script demonstrates:
# 1) loading trajectories from CSV
# 2) timestamp association + linear interpolation
# 3) ATE and RPE metrics
# 4) normalized benchmark score aggregation
# 5) a simple leakage checker for train/val/test splits

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Tuple, Dict, List

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


@dataclass
class TrajectorySE2:
    t: np.ndarray  # shape (N,)
    x: np.ndarray  # shape (N,)
    y: np.ndarray  # shape (N,)
    yaw: np.ndarray  # shape (N,)

    @classmethod
    def from_csv(cls, path: str) -> "TrajectorySE2":
        df = pd.read_csv(path)
        required = {"t", "x", "y", "yaw"}
        if not required.issubset(df.columns):
            raise ValueError(f"{path} must contain columns {sorted(required)}")
        df = df.sort_values("t").reset_index(drop=True)
        return cls(
            t=df["t"].to_numpy(float),
            x=df["x"].to_numpy(float),
            y=df["y"].to_numpy(float),
            yaw=df["yaw"].to_numpy(float),
        )

    def to_matrix(self) -> np.ndarray:
        # Returns N x 3 matrix [x, y, yaw]
        return np.column_stack([self.x, self.y, self.yaw])


def wrap_angle(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def interp_se2(ref: TrajectorySE2, tq: np.ndarray) -> np.ndarray:
    """
    Linear interpolation in x, y and wrapped interpolation in yaw.
    Returns array of shape (len(tq), 3).
    """
    if np.any(tq < ref.t[0]) or np.any(tq > ref.t[-1]):
        raise ValueError("Query timestamps must lie inside reference interval")

    xq = np.interp(tq, ref.t, ref.x)
    yq = np.interp(tq, ref.t, ref.y)

    # Wrapped yaw interpolation: interpolate sin/cos then recover angle
    c = np.interp(tq, ref.t, np.cos(ref.yaw))
    s = np.interp(tq, ref.t, np.sin(ref.yaw))
    yawq = np.arctan2(s, c)
    return np.column_stack([xq, yq, yawq])


def rigid_align_2d(est_xy: np.ndarray, gt_xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Umeyama-style rigid alignment without scale in 2D.
    est_xy, gt_xy: N x 2 arrays.
    Returns (R, t) so that aligned = est_xy @ R.T + t is aligned to gt_xy.
    """
    if est_xy.shape != gt_xy.shape or est_xy.shape[1] != 2:
        raise ValueError("Inputs must both be N x 2")
    mu_e = est_xy.mean(axis=0)
    mu_g = gt_xy.mean(axis=0)
    E = est_xy - mu_e
    G = gt_xy - mu_g
    H = E.T @ G
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T
    t = mu_g - mu_e @ R.T
    return R, t


def compute_ate_rmse(gt: TrajectorySE2, est: TrajectorySE2, align: bool = True) -> Dict[str, float]:
    # Associate by estimator timestamps (common in benchmark tools after synchronization)
    gt_interp = interp_se2(gt, est.t)
    est_pose = est.to_matrix().copy()

    if align:
        R, t = rigid_align_2d(est_pose[:, :2], gt_interp[:, :2])
        est_pose[:, :2] = est_pose[:, :2] @ R.T + t

    e_xy = est_pose[:, :2] - gt_interp[:, :2]
    e_yaw = wrap_angle(est_pose[:, 2] - gt_interp[:, 2])

    trans_rmse = float(np.sqrt(np.mean(np.sum(e_xy**2, axis=1))))
    yaw_rmse = float(np.sqrt(np.mean(e_yaw**2)))
    return {"ATE_trans_rmse_m": trans_rmse, "ATE_yaw_rmse_rad": yaw_rmse}


def relative_motion(pose: np.ndarray, idx0: int, idx1: int) -> np.ndarray:
    """
    pose: N x 3 [x, y, yaw], returns relative SE2 as [dx_body, dy_body, dyaw]
    """
    x0, y0, th0 = pose[idx0]
    x1, y1, th1 = pose[idx1]
    dxw = x1 - x0
    dyw = y1 - y0
    c = math.cos(th0)
    s = math.sin(th0)
    dx = c * dxw + s * dyw
    dy = -s * dxw + c * dyw
    dth = ((th1 - th0 + np.pi) % (2*np.pi)) - np.pi
    return np.array([dx, dy, dth], dtype=float)


def compute_rpe(gt: TrajectorySE2, est: TrajectorySE2, delta_t: float = 1.0) -> Dict[str, float]:
    gt_pose = interp_se2(gt, est.t)
    est_pose = est.to_matrix()
    pairs: List[Tuple[int, int]] = []
    for i in range(len(est.t)):
        target_t = est.t[i] + delta_t
        j = int(np.searchsorted(est.t, target_t))
        if j < len(est.t):
            pairs.append((i, j))
    if not pairs:
        raise ValueError("No valid pairs for the requested delta_t")

    e_trans = []
    e_yaw = []
    for i, j in pairs:
        d_gt = relative_motion(gt_pose, i, j)
        d_est = relative_motion(est_pose, i, j)
        diff = d_est - d_gt
        diff[2] = ((diff[2] + np.pi) % (2*np.pi)) - np.pi
        e_trans.append(float(np.linalg.norm(diff[:2])))
        e_yaw.append(float(abs(diff[2])))

    return {
        "RPE_trans_mean_m": float(np.mean(e_trans)),
        "RPE_trans_rmse_m": float(np.sqrt(np.mean(np.square(e_trans)))),
        "RPE_yaw_mean_rad": float(np.mean(e_yaw)),
        "num_pairs": len(pairs),
    }


def normalized_score(metrics: Dict[str, float], caps: Dict[str, float]) -> float:
    """
    Convert a dictionary of 'lower-is-better' metrics into [0,100].
    score_j = max(0, 1 - m_j / cap_j)
    final score = 100 * weighted average (uniform weights here)
    """
    vals = []
    for k, cap in caps.items():
        if cap <= 0:
            raise ValueError(f"cap for {k} must be positive")
        m = metrics[k]
        vals.append(max(0.0, 1.0 - m / cap))
    return 100.0 * float(np.mean(vals))


def leakage_check(splits: pd.DataFrame) -> pd.DataFrame:
    """
    splits columns: sequence_id, split in {train,val,test}
    Flags sequences appearing in more than one split.
    """
    bad = (
        splits.groupby("sequence_id")["split"]
        .nunique()
        .reset_index(name="num_splits")
        .query("num_splits > 1")
    )
    return bad


def make_demo_data(out_dir: Path, n: int = 600, dt: float = 0.1) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    t = np.arange(n) * dt
    # Ground truth: smooth path
    x = 0.5 * t + 2.0*np.sin(0.15*t)
    y = 1.5*np.cos(0.10*t)
    yaw = np.arctan2(np.gradient(y, dt), np.gradient(x, dt))
    gt = pd.DataFrame({"t": t, "x": x, "y": y, "yaw": yaw})

    # Estimated trajectory: drift + noise + slight time warp
    t_est = t.copy()
    t_est = t_est + 0.01*np.sin(0.05*t_est)  # small timestamp distortion
    x_e = x + 0.03*t + 0.08*np.random.randn(n)
    y_e = y - 0.02*t + 0.08*np.random.randn(n)
    yaw_e = yaw + np.deg2rad(2.0)*np.sin(0.2*t) + np.deg2rad(1.0)*np.random.randn(n)
    est = pd.DataFrame({"t": t_est, "x": x_e, "y": y_e, "yaw": yaw_e})

    gt.to_csv(out_dir / "gt_traj.csv", index=False)
    est.to_csv(out_dir / "est_traj.csv", index=False)

    splits = pd.DataFrame({
        "sequence_id": ["S01", "S02", "S03", "S02", "S04"],
        "split": ["train", "train", "val", "test", "test"]
    })
    splits.to_csv(out_dir / "splits.csv", index=False)


def main() -> None:
    np.random.seed(7)
    data_dir = Path("benchmark_demo")
    make_demo_data(data_dir)

    gt = TrajectorySE2.from_csv(str(data_dir / "gt_traj.csv"))
    est = TrajectorySE2.from_csv(str(data_dir / "est_traj.csv"))

    ate = compute_ate_rmse(gt, est, align=True)
    rpe = compute_rpe(gt, est, delta_t=1.0)
    print("ATE:", ate)
    print("RPE:", rpe)

    caps = {
        "ATE_trans_rmse_m": 2.0,
        "RPE_trans_rmse_m": 0.8,
    }
    score = normalized_score(
        {"ATE_trans_rmse_m": ate["ATE_trans_rmse_m"], "RPE_trans_rmse_m": rpe["RPE_trans_rmse_m"]},
        caps
    )
    print(f"Normalized aggregate score: {score:.2f}/100")

    splits = pd.read_csv(data_dir / "splits.csv")
    bad = leakage_check(splits)
    if len(bad) > 0:
        print("\nData leakage warning:")
        print(bad.to_string(index=False))

    # Visualization
    gt_interp = interp_se2(gt, est.t)
    R, tvec = rigid_align_2d(est.to_matrix()[:, :2], gt_interp[:, :2])
    est_aligned = est.to_matrix().copy()
    est_aligned[:, :2] = est_aligned[:, :2] @ R.T + tvec
    e = est_aligned[:, :2] - gt_interp[:, :2]
    e_norm = np.linalg.norm(e, axis=1)

    plt.figure(figsize=(7, 5))
    plt.plot(gt.x, gt.y, label="Ground truth")
    plt.plot(est_aligned[:, 0], est_aligned[:, 1], label="Estimate (aligned)")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajectory Comparison")
    plt.axis("equal")
    plt.legend()
    plt.tight_layout()
    plt.savefig(data_dir / "trajectory_compare.png", dpi=140)

    plt.figure(figsize=(7, 3))
    plt.plot(est.t, e_norm)
    plt.xlabel("time [s]")
    plt.ylabel("pos error [m]")
    plt.title("Instantaneous Position Error")
    plt.tight_layout()
    plt.savefig(data_dir / "error_over_time.png", dpi=140)
    plt.show()


if __name__ == "__main__":
    main()
