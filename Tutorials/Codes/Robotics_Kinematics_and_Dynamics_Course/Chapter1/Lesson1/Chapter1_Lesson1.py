import numpy as np

# Example: simple planar constraint relating joint velocities to an end-effector velocity
# Suppose A * qdot = vx, where qdot in R^3 and vx in R^2
A = np.array([[1.0,  1.0, 0.0],
              [0.0,  1.0, 1.0]])

print("A =\n", A)

# Numerical rank using NumPy (SVD-based)
rank = np.linalg.matrix_rank(A)
print("rank(A) =", rank)

# Approximate null space using SVD
def null_space(A, tol=1e-10):
    U, S, Vt = np.linalg.svd(A)
    # Singular values close to zero correspond to null space directions
    mask = S < tol
    # Rows of Vt corresponding to small singular values span the null space
    # (Note: Vt has shape (n, n) for full SVD)
    return Vt[mask].T

N = null_space(A)
print("Null space basis (columns):\n", N)

# Check that A * z is numerically close to zero for each basis vector z
for i in range(N.shape[1]):
    z = N[:, i]
    print(f"Check A @ z_{i} =", A @ z)
      
