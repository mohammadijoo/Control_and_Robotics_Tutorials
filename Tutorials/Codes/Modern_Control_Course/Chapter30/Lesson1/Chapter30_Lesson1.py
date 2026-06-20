"""
Chapter30_Lesson1.py
Descriptor (singular) system example: E xdot = A x + B u.

This script demonstrates:
1. generalized eigenvalue analysis of the pencil lambda E - A,
2. index-1 algebraic reduction when E = diag(I, 0),
3. simulation of the reduced ODE and reconstruction of the algebraic state.

Required packages: numpy, scipy, matplotlib
"""

import numpy as np
from scipy.linalg import eig
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Descriptor model with two differential states and one algebraic state.
# E xdot = A x + B u
E = np.diag([1.0, 1.0, 0.0])
A = np.array([
    [0.0,  1.0, 0.0],
    [-2.0, -3.0, 1.0],
    [1.0,  0.0, 1.0],
])
B = np.array([[0.0], [1.0], [-1.0]])

# Generalized eigenvalues solve det(lambda E - A) = 0.
finite_and_infinite_eigs = eig(A, E, right=False)
print("Generalized eigenvalues of (A, E):")
print(finite_and_infinite_eigs)

# Partition x = [xd; xa], where xd = [x1, x2]^T and xa = x3.
A11 = A[:2, :2]
A12 = A[:2, 2:3]
A21 = A[2:3, :2]
A22 = A[2:3, 2:3]
B1 = B[:2, :]
B2 = B[2:3, :]

# Algebraic equation: 0 = A21 xd + A22 xa + B2 u.
# If A22 is nonsingular, xa = -A22^{-1} A21 xd - A22^{-1} B2 u.
A22_inv = np.linalg.inv(A22)
Ar = A11 - A12 @ A22_inv @ A21
Br = B1 - A12 @ A22_inv @ B2

print("Reduced ODE matrix Ar:")
print(Ar)
print("Reduced input matrix Br:")
print(Br)
print("Eigenvalues of the reduced dynamics:")
print(np.linalg.eigvals(Ar))


def u_of_t(t: float) -> float:
    """Piecewise-smooth input."""
    return 1.0 if t >= 1.0 else 0.0


def rhs(t: float, xd: np.ndarray) -> np.ndarray:
    u = np.array([[u_of_t(t)]])
    return (Ar @ xd.reshape(2, 1) + Br @ u).ravel()


def reconstruct_xa(xd: np.ndarray, u: float) -> float:
    xd_col = xd.reshape(2, 1)
    xa = -A22_inv @ A21 @ xd_col - A22_inv @ B2 * u
    return float(xa[0, 0])

# Consistent initial condition: choose xd(0), then compute xa(0) from constraint.
xd0 = np.array([0.2, 0.0])
xa0 = reconstruct_xa(xd0, u_of_t(0.0))
print("Consistent initial x0 =", np.r_[xd0, xa0])

sol = solve_ivp(rhs, (0.0, 8.0), xd0, t_eval=np.linspace(0.0, 8.0, 401))
x1 = sol.y[0, :]
x2 = sol.y[1, :]
x3 = np.array([reconstruct_xa(sol.y[:, k], u_of_t(sol.t[k])) for k in range(sol.t.size)])
constraint_residual = A21 @ sol.y + A22 @ x3.reshape(1, -1) + B2 @ np.array([[u_of_t(t) for t in sol.t]])
print("Maximum algebraic constraint residual:", np.max(np.abs(constraint_residual)))

plt.figure(figsize=(8, 4.8))
plt.plot(sol.t, x1, label="x1 differential")
plt.plot(sol.t, x2, label="x2 differential")
plt.plot(sol.t, x3, label="x3 algebraic")
plt.xlabel("time")
plt.ylabel("state")
plt.title("Descriptor system reduced to an index-1 ODE")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
