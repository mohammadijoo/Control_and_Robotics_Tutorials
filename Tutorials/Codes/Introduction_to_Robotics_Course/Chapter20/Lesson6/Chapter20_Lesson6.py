import numpy as np
import pandas as pd

def compute_run_metrics(df_run):
    t = df_run["t"].to_numpy()
    r = df_run["r"].to_numpy()
    y = df_run["y"].to_numpy()
    u = df_run["u"].to_numpy()
    # Assume t is strictly increasing
    dt = np.diff(t, prepend=t[0])
    e = r - y

    rms = np.sqrt(np.mean(e**2))
    iae = np.sum(np.abs(e) * dt)
    u2 = np.sum(u**2 * dt)
    violation_rate = df_run["constraint_violation"].mean()
    return {
        "rms": rms,
        "iae": iae,
        "u2": u2,
        "violation_rate": violation_rate,
    }

# Load a single iteration's log (all runs for iteration k)
df = pd.read_csv("robot_logs_iteration_k.csv")

grouped = df.groupby("run_id")
rows = []
for run_id, df_run in grouped:
    rows.append({"run_id": run_id, **compute_run_metrics(df_run)})

run_metrics = pd.DataFrame(rows)

# Aggregate across runs
E_rms_bar = run_metrics["rms"].mean()
U2_bar = run_metrics["u2"].mean()
V_bar = run_metrics["violation_rate"].mean()

# Normalization constants and targets
target_rms = 0.01
scale_rms = 0.05
target_u2 = 0.0
scale_u2 = max(U2_bar, 1e-3)
target_v = 0.0
scale_v = 1.0

tilde_rms = (E_rms_bar - target_rms) / scale_rms
tilde_u2 = (U2_bar - target_u2) / scale_u2
tilde_v = (V_bar - target_v) / scale_v

# Weights: emphasize safety, then tracking, then effort
w_rms = 0.3
w_u2 = 0.2
w_v = 0.5

J_k = w_rms * tilde_rms + w_u2 * tilde_u2 + w_v * tilde_v
print("Iteration score J_k =", J_k)
      
