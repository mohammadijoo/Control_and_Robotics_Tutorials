import numpy as np

# Assume we have logged arrays of joint states and measured torque:
# q, qd, qdd, tau_meas are 1D NumPy arrays of length N
# q      : joint position [rad]
# qd     : joint velocity [rad/s]
# qdd    : joint acceleration [rad/s^2]
# tau_meas : measured motor torque [Nm]

def build_regressor_pendulum(q, qd, qdd, g=9.81):
    """
    Build the regressor matrix Y for the 1-DOF pendulum:
      tau = a1 * qdd + a2 * sin(q) + a3 * qd
    Parameters:
      q, qd, qdd : shape (N,)
    Returns:
      Y : shape (N, 3)
    """
    q = np.asarray(q)
    qd = np.asarray(qd)
    qdd = np.asarray(qdd)
    Y = np.column_stack([qdd, np.sin(q), qd])
    return Y

# Example: synthetic data with known parameters
N = 1000
t = np.linspace(0.0, 10.0, N)
q = 0.5 * np.sin(2.0 * t)  # simple excitation
qd = np.gradient(q, t)
qdd = np.gradient(qd, t)

phi_true = np.array([0.08, 2.5, 0.1])  # [a1, a2, a3]
Y = build_regressor_pendulum(q, qd, qdd)
tau_clean = Y @ phi_true
noise_std = 0.05
tau_meas = tau_clean + noise_std * np.random.randn(N)

# Least squares estimate using numpy.linalg.lstsq
phi_hat, residuals, rank, s = np.linalg.lstsq(Y, tau_meas, rcond=None)

print("True parameters:     ", phi_true)
print("Estimated parameters:", phi_hat)
print("Rank of Y:", rank)

# Predicted torques for diagnostics
tau_pred = Y @ phi_hat
rmse = np.sqrt(np.mean((tau_pred - tau_meas) ** 2))
print("RMSE on training data:", rmse)
      
