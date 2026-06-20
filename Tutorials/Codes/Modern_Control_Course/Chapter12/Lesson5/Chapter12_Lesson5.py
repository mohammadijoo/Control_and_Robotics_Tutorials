# Chapter12_Lesson5.py
# Balanced-realization preview for a continuous-time stable LTI system.
# Requires: numpy, scipy

import numpy as np
from scipy.linalg import solve_continuous_lyapunov, cholesky, eigh

np.set_printoptions(precision=6, suppress=True)

# x_dot = A x + B u, y = C x
# A is Hurwitz, so infinite-horizon Gramians are finite.
A = np.array([[-1.0, 0.3],
              [ 0.0,-2.0]])
B = np.array([[1.0],
              [0.5]])
C = np.array([[1.0, -0.2]])

# Controllability Gramian:
# A Wc + Wc A^T + B B^T = 0
Wc = solve_continuous_lyapunov(A, -(B @ B.T))

# Output-energy / observability Gramian preview:
# A^T Wo + Wo A + C^T C = 0
Wo = solve_continuous_lyapunov(A.T, -(C.T @ C))

# Symmetrize against floating-point roundoff.
Wc = 0.5 * (Wc + Wc.T)
Wo = 0.5 * (Wo + Wo.T)

# Hankel singular values are sqrt(eig(Wc Wo)).
# A numerically stable computation uses Cholesky(Wc) and eig(R^T Wo R).
R = cholesky(Wc, lower=True)
M = R.T @ Wo @ R
eigvals, U = eigh(M)
idx = np.argsort(eigvals)[::-1]
eigvals = eigvals[idx]
U = U[:, idx]
sigma = np.sqrt(np.maximum(eigvals, 0.0))
Sigma = np.diag(sigma)

# Balancing transformation for x = T z.
# In z-coordinates:
# Wc_z = T^{-1} Wc T^{-T}, Wo_z = T^T Wo T.
T = R @ U @ np.diag(1.0 / np.sqrt(sigma))
Tinv = np.linalg.inv(T)

Ab = Tinv @ A @ T
Bb = Tinv @ B
Cb = C @ T
Wc_bal = Tinv @ Wc @ Tinv.T
Wo_bal = T.T @ Wo @ T

print("A =\n", A)
print("B =\n", B)
print("C =\n", C)
print("\nControllability Gramian Wc =\n", Wc)
print("\nOutput-energy Gramian Wo =\n", Wo)
print("\nHankel singular values sigma =\n", sigma)
print("\nBalancing transformation T for x = T z =\n", T)
print("\nBalanced A =\n", Ab)
print("Balanced B =\n", Bb)
print("Balanced C =\n", Cb)
print("\nWc in balanced coordinates =\n", Wc_bal)
print("\nWo in balanced coordinates =\n", Wo_bal)
print("\nTarget diagonal Sigma =\n", Sigma)

# Energy check for one final state.
x_final = np.array([[1.0], [0.2]])
z_final = Tinv @ x_final
energy_original = float(x_final.T @ np.linalg.inv(Wc) @ x_final)
energy_balanced = float(z_final.T @ np.linalg.inv(Wc_bal) @ z_final)
print("\nMinimum input energy to reach x_final:", energy_original)
print("Same energy computed in balanced coordinates:", energy_balanced)
