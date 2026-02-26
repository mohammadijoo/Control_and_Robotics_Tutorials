import pandas as pd
import numpy as np

# Load CSV exported from simulator or robot logger
df = pd.read_csv("robot_logs.csv")

# Compute error signal
df["e"] = df["y_ref"] - df["y_meas"]

# Group by trial and compute RMS error
def rms(x):
    return np.sqrt(np.mean(np.square(x)))

rms_per_trial = df.groupby("trial_id")["e"].apply(rms)

J_values = rms_per_trial.values
N = len(J_values)

J_mean = np.mean(J_values)
J_std = np.std(J_values, ddof=1)

print("RMS error per trial:")
print(rms_per_trial)
print(f"Mean RMS error: {J_mean:.4f}")
print(f"Std of RMS error: {J_std:.4f}")

# 95% confidence interval for the mean (approximate, using normal quantile)
alpha = 0.05
z = 1.96  # approximate for large N
se = J_std / np.sqrt(N)
print(f"95% CI for mean RMS error: [{J_mean - z * se:.4f}, {J_mean + z * se:.4f}]")
      
