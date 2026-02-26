import numpy as np
import pandas as pd
from scipy import stats

# Load experimental log
df = pd.read_csv("planner_logs.csv")  # columns: method, success, cost

def summarize_method(df_method, alpha=0.05):
    """Return success rate CI and cost CI for one method."""
    successes = df_method["success"].to_numpy()
    costs = df_method["cost"].to_numpy()

    N = len(successes)
    p_hat = successes.mean()
    # Normal approximation CI for Bernoulli
    z = stats.norm.ppf(1 - alpha / 2.0)
    se_p = np.sqrt(p_hat * (1.0 - p_hat) / N)
    ci_p = (p_hat - z * se_p, p_hat + z * se_p)

    # t-based CI for mean cost
    N_cost = len(costs)
    c_bar = costs.mean()
    s_c = costs.std(ddof=1)
    t_val = stats.t.ppf(1 - alpha / 2.0, df=N_cost - 1)
    se_c = s_c / np.sqrt(N_cost)
    ci_c = (c_bar - t_val * se_c, c_bar + t_val * se_c)

    return {
        "N": N,
        "p_hat": p_hat,
        "ci_p": ci_p,
        "c_bar": c_bar,
        "ci_c": ci_c,
    }

# Separate methods
methods = df["method"].unique()
summary = {m: summarize_method(df[df["method"] == m]) for m in methods}

for m, stats_m in summary.items():
    print(f"Method {m}:")
    print(f"  N trials       = {stats_m['N']}")
    print(f"  success rate   = {stats_m['p_hat']:.3f}, "
          f"CI = ({stats_m['ci_p'][0]:.3f}, {stats_m['ci_p'][1]:.3f})")
    print(f"  mean cost      = {stats_m['c_bar']:.3f}, "
          f"CI = ({stats_m['ci_c'][0]:.3f}, {stats_m['ci_c'][1]:.3f})")

# Two-proportion z-test for success rate difference (new vs baseline)
baseline = "baseline"
new = "new"

df_A = df[df["method"] == new]
df_B = df[df["method"] == baseline]

S_A = df_A["success"].to_numpy()
S_B = df_B["success"].to_numpy()
N_A, N_B = len(S_A), len(S_B)
p_A, p_B = S_A.mean(), S_B.mean()

p_pool = (N_A * p_A + N_B * p_B) / (N_A + N_B)
se_diff = np.sqrt(p_pool * (1.0 - p_pool) * (1.0 / N_A + 1.0 / N_B))
Z = (p_A - p_B) / se_diff
p_value_one_sided = 1.0 - stats.norm.cdf(Z)

print("\nTwo-proportion z-test (success rate):")
print(f"  p_A = {p_A:.3f}, p_B = {p_B:.3f}")
print(f"  Z   = {Z:.3f}, one-sided p-value = {p_value_one_sided:.4f}")

# t-test for mean cost (lower is better)
cost_A = df_A["cost"].to_numpy()
cost_B = df_B["cost"].to_numpy()

t_stat, p_val_cost = stats.ttest_ind(cost_B, cost_A, equal_var=False)
print("\nWelch t-test on cost (H1: new method has lower cost):")
print(f"  t = {t_stat:.3f}, two-sided p-value = {p_val_cost:.4f}")
      
