
import numpy as np
import pandas as pd

# Load log
df = pd.read_csv("demo_log.csv")

t = df["t"].to_numpy()
Ts = np.mean(np.diff(t))

# Extract joint trajectories as matrices of shape (N, n)
q_cols = [c for c in df.columns if c.startswith("q_")]
qd_cols = [c for c in df.columns if c.startswith("qd_")]
tau_cols = [c for c in df.columns if c.startswith("tau_")]

q = df[q_cols].to_numpy()
qd = df[qd_cols].to_numpy()
tau = df[tau_cols].to_numpy()

e = qd - q

def ise(e, Ts):
    """
    Integral of squared error, approximated by Riemann sum.
    e: array of shape (N, n)
    """
    # Skip last sample to match Ts intervals
    return Ts * np.sum(np.sum(e[:-1, :] ** 2, axis=1))

def iae(e, Ts):
    """
    Integral of absolute error.
    """
    return Ts * np.sum(np.sum(np.abs(e[:-1, :]), axis=1))

def rms_per_joint(e, Ts):
    """
    RMS error per joint using integral approximation.
    """
    T = Ts * (e.shape[0] - 1)
    sq_int = Ts * np.sum(e[:-1, :] ** 2, axis=0)
    return np.sqrt(sq_int / T)

def quadratic_effort(tau, Ts, R=None):
    """
    Quadratic effort J_u = integral tau^T R tau dt.
    If R is None, use identity weighting.
    """
    if R is None:
        R = np.eye(tau.shape[1])
    # Apply R to each sample
    weighted = np.einsum("ij,jk,ik->i", tau[:-1, :], R, tau[:-1, :])
    return Ts * np.sum(weighted)

J_ISE = ise(e, Ts)
J_IAE = iae(e, Ts)
rms_joints = rms_per_joint(e, Ts)
J_u = quadratic_effort(tau, Ts)

print("ISE:", J_ISE)
print("IAE:", J_IAE)
print("RMS per joint:", rms_joints)
print("Quadratic effort J_u:", J_u)
