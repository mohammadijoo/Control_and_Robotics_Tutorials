import numpy as np

# Sampling grid
T_s = 0.002  # 500 Hz
T_end = 8.0
t = np.arange(0.0, T_end, T_s)
N = t.size

# True (grouped) parameters: pi_1, pi_2, pi_3
pi_true = np.array([0.35, 2.1, 0.08])

# Exciting joint trajectory (smooth, multi-sine)
q = 0.6 * np.sin(2.0 * np.pi * 0.4 * t) + 0.4 * np.sin(2.0 * np.pi * 0.9 * t)

# Compute derivatives analytically for clean ground truth
dq = 0.6 * 2.0 * np.pi * 0.4 * np.cos(2.0 * np.pi * 0.4 * t) \
   + 0.4 * 2.0 * np.pi * 0.9 * np.cos(2.0 * np.pi * 0.9 * t)
ddq = -0.6 * (2.0 * np.pi * 0.4)**2 * np.sin(2.0 * np.pi * 0.4 * t) \
    - 0.4 * (2.0 * np.pi * 0.9)**2 * np.sin(2.0 * np.pi * 0.9 * t)

# Build noiseless torque from regressor form: tau = Y pi
Y = np.column_stack([ddq, np.cos(q), dq])
tau_clean = Y @ pi_true

# Add measurement noise (e.g. encoder and current sensor noise)
rng = np.random.default_rng(1)
tau_meas = tau_clean + 0.05 * rng.standard_normal(N)

# Estimate parameters using least squares
# Use a burn-in at both ends to avoid transient filtering issues
Y_id = Y[50:-50, :]
tau_id = tau_meas[50:-50]

# Normal equations solution
YTY = Y_id.T @ Y_id
YTT = Y_id.T @ tau_id
pi_hat = np.linalg.solve(YTY, YTT)

print("True parameters :", pi_true)
print("Estimated       :", pi_hat)
print("Relative error  :", (pi_hat - pi_true) / pi_true)

# Simple validation on held-out subset
Y_val = Y[:50, :]
tau_val_true = tau_clean[:50]
tau_val_pred = Y_val @ pi_hat
rms_val = np.sqrt(np.mean((tau_val_pred - tau_val_true)**2))
print("Validation RMS torque error:", rms_val)

# (Optional, robotics toolbox) - structure for a real robot
# from roboticstoolbox import DHRobot, RevoluteDH
# robot = DHRobot([RevoluteDH(a=0.5, alpha=0.0, d=0.0)])
# For each time sample: robot.rne(qk, dqk, ddqk) gives tau_k using built-in dynamics.
      
