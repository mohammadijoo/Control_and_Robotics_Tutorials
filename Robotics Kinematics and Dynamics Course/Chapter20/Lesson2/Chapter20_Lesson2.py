import numpy as np

# Suppose we have a function simulate_robot that returns joint trajectories
# using, for example, pinocchio or roboticstoolbox.
# q_traj: shape (K, n), each row is q(t_k).
def simulate_robot(num_samples=1000):
    # Placeholder: in practice, call your dynamics integrator here.
    # For illustration, make a correlated random trajectory.
    n = 12  # e.g. 12-DOF arm
    t = np.linspace(0.0, 10.0, num_samples)
    base_signals = np.stack([
        np.sin(0.5 * t),
        np.cos(0.3 * t),
        np.sin(0.8 * t + 0.5),
        np.cos(0.2 * t + 1.0)
    ], axis=1)  # (K, 4)
    W = np.random.randn(4, n)  # mixing matrix
    q_traj = base_signals @ W  # (K, n)
    return q_traj

q_traj = simulate_robot(num_samples=2000)   # (K, n)
K, n = q_traj.shape

# 1) Center data
q_mean = np.mean(q_traj, axis=0, keepdims=True)  # (1, n)
Qc = q_traj - q_mean                             # (K, n)

# 2) SVD
U, S, Vt = np.linalg.svd(Qc, full_matrices=False)
# S: (min(K,n),), Vt: (n, n)

# 3) Choose reduced dimension r by energy threshold
energy = np.cumsum(S**2) / np.sum(S**2)
r = int(np.searchsorted(energy, 0.99) + 1)  # smallest r with 99% energy
V_r = Vt[:r, :].T  # (n, r) basis as columns

print(f"Full dimension n = {n}, reduced r = {r}")

def project(q):
    """
    Project joint configuration q (n,) to reduced coords z (r,).
    """
    q = np.asarray(q).reshape(1, -1)
    return (q - q_mean) @ V_r  # (1, r)

def reconstruct(z):
    """
    Reconstruct joint configuration from reduced coords z.
    """
    z = np.asarray(z).reshape(1, -1)
    return q_mean + z @ V_r.T

# Example: take a sample from the trajectory and project/reconstruct
sample_q = q_traj[100]
z = project(sample_q)
q_rec = reconstruct(z).ravel()

print("Projection error norm:",
      np.linalg.norm(sample_q - q_rec))
      
