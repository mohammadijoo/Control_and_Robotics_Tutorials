import numpy as np

g = 9.81

def pendulum_regressor_row(q, qdd):
    """
    Regressor row for the over-parameterized pendulum model:
    tau = [qdd, 0, qdd, g*sin(q)] @ [I0, m, m*c**2, m*c]
    """
    return np.array([qdd, 0.0, qdd, g * np.sin(q)], dtype=float)

# Generate random samples of (q, qdd)
rng = np.random.default_rng(0)
N = 50
qs = rng.uniform(low=-np.pi, high=np.pi, size=N)
qdds = rng.uniform(low=-5.0, high=5.0, size=N)

# Stack into big regressor W (size N x 4)
W = np.vstack([pendulum_regressor_row(q, qdd) for q, qdd in zip(qs, qdds)])

# Compute SVD of W
U, S, Vt = np.linalg.svd(W, full_matrices=False)

print("Singular values:", S)

# Numerical rank: count singular values above a tolerance
tol = 1e-8
r = np.sum(S > tol)
print("Numerical rank r =", r)

V1 = Vt[:r, :].T   # first r right-singular vectors (columns)
V2 = Vt[r:, :].T   # remaining p-r columns (null space basis)

print("V1 (basis of base parameter space):\n", V1)
print("V2 (basis of null space):\n", V2)

# Transform a sample full parameter vector pi into base parameters beta:
pi_full = np.array([0.1, 2.0, 2.0 * 0.25, 2.0 * 0.5])  # I0, m, m*c**2, m*c
beta = V1.T @ pi_full
print("Base parameter coordinates beta:", beta)
      
