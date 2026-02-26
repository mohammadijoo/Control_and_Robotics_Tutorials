
import numpy as np
from numpy.linalg import eigvals
from scipy.linalg import solve_continuous_lyapunov

# Example 2-DOF joint error dynamics: x = [q1, q2, dq1, dq2]
# A could come from linearizing the full rigid-body dynamics
A = np.array([
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [-50.0, 0.0, -10.0, 0.0],
    [0.0, -40.0, 0.0, -12.0],
])

# Check eigenvalues (Hurwitz if all real parts < 0)
lam = eigvals(A)
print("Eigenvalues of A:", lam)

# Choose Q > 0 (e.g., identity)
Q = np.eye(4)

# Solve A^T P + P A = -Q for P
P = solve_continuous_lyapunov(A, -Q)

# Check symmetry and definiteness of P
print("P (Lyapunov solution):\n", P)
print("Eigenvalues of P:", eigvals(P))

def V(x):
    return float(x.T @ P @ x)

def dVdt(x):
    # For linear system, dV/dt = x^T (A^T P + P A) x = -x^T Q x
    return float(-x.T @ Q @ x)

# Example trajectory simulation
def simulate(x0, dt=0.001, T=2.0):
    x = x0.copy()
    xs = [x.copy()]
    ts = [0.0]
    t = 0.0
    while t < T:
        xdot = A @ x
        x = x + dt * xdot
        t += dt
        xs.append(x.copy())
        ts.append(t)
    return np.array(ts), np.array(xs)

x0 = np.array([0.1, -0.1, 0.0, 0.0])
ts, xs = simulate(x0)

print("Initial V:", V(x0))
print("Final V:", V(xs[-1]))
