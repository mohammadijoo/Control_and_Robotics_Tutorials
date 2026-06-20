import numpy as np

A = np.array([[1., 2., 3., 1.],
              [2., 4., 6., 2.],
              [1., 1., 1., 0.]], dtype=float)

# Numerical rank (SVD-based internally)
rank_A = np.linalg.matrix_rank(A, tol=1e-10)
print("rank(A) =", rank_A)

# Null space via SVD (from scratch)
U, s, Vt = np.linalg.svd(A)
tol = 1e-10
null_mask = (s < tol)
# For rectangular matrices, s has length min(m,n); we need null vectors from Vt
# Null space dimension = n - rank
n = A.shape[1]
k = n - rank_A
null_basis = Vt[-k:, :].T  # columns span null space

print("nullity(A) =", null_basis.shape[1])
print("rank + nullity =", rank_A + null_basis.shape[1], " (should equal n =", n, ")")

# Verification
residuals = A @ null_basis
print("||A * N||_F =", np.linalg.norm(residuals, ord="fro"))

# Exact rational null space (symbolic) using SymPy
import sympy as sp
As = sp.Matrix([[1,2,3,1],
                [2,4,6,2],
                [1,1,1,0]])
print("rank_sym(A) =", As.rank())
print("nullspace_sym(A) =", As.nullspace())
