import pandas as pd
import numpy as np
from scipy import stats

# Load benchmark log produced by e.g. OMPL or MoveIt
df = pd.read_csv("planning_benchmark.csv")

# Basic utility: confidence interval for the mean (continuous metric)
def mean_ci(x, alpha=0.05):
    x = np.asarray(x, dtype=float)
    n = x.size
    mean = x.mean()
    std = x.std(ddof=1)
    tval = stats.t.ppf(1.0 - alpha / 2.0, df=n - 1)
    half_width = tval * std / np.sqrt(n)
    return mean, mean - half_width, mean + half_width

# Success rate CI (normal approximation)
def success_rate_ci(successes, alpha=0.05):
    x = np.asarray(successes, dtype=float)
    n = x.size
    p_hat = x.mean()
    z = stats.norm.ppf(1.0 - alpha / 2.0)
    half_width = z * np.sqrt(p_hat * (1.0 - p_hat) / n)
    return p_hat, p_hat - half_width, p_hat + half_width

planners = df["planner"].unique()
print("Planners:", planners)

# Aggregate metrics per planner
for planner in planners:
    sub = df[df["planner"] == planner]
    succ, succ_lo, succ_hi = success_rate_ci(sub["success"])
    plen, plen_lo, plen_hi = mean_ci(sub[sub["success"] == 1]["path_length"])
    tmean, t_lo, t_hi = mean_ci(sub[sub["success"] == 1]["solve_time"])
    print(f"Planner {planner}")
    print(f"  Success rate: {succ:.3f} [{succ_lo:.3f}, {succ_hi:.3f}]")
    print(f"  Path length (success only): {plen:.3f} [{plen_lo:.3f}, {plen_hi:.3f}]")
    print(f"  Solve time (success only): {tmean:.3f} [{t_lo:.3f}, {t_hi:.3f}]")

# Paired comparison between two planners on the same (scene_id, seed)
planner_a, planner_b = planners[:2]

pivot = df.pivot_table(
    index=["scene_id", "seed"],
    columns="planner",
    values=["success", "path_length"]
)

# Paired differences for path length where both succeeded
mask_both = (pivot["success"][planner_a] == 1) & (pivot["success"][planner_b] == 1)
c1 = pivot["path_length"][planner_a][mask_both]
c2 = pivot["path_length"][planner_b][mask_both]
d = c1.values - c2.values
t_stat, p_val = stats.ttest_1samp(d, popmean=0.0)
print(f"Paired t-test on path length ({planner_a} - {planner_b}):")
print(f"  n_pairs = {d.size}, t = {t_stat:.3f}, p = {p_val:.3g}")
      
