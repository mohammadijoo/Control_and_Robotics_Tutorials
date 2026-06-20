# Chapter19_Lesson5.py
# Spatio-Temporal Dynamics: 1D heat equation discretization + (finite-dimensional) LQR

import numpy as np
from scipy.linalg import solve_continuous_are

def laplacian_1d_dirichlet(N: int, L: float):
    """Second-derivative matrix on N interior points with Dirichlet boundaries."""
    dx = L / (N + 1)
    main = -2.0 * np.ones(N)
    off  =  1.0 * np.ones(N - 1)
    D2 = (np.diag(main) + np.diag(off, 1) + np.diag(off, -1)) / (dx * dx)
    x = dx * (np.arange(1, N + 1))  # interior grid
    return D2, x, dx

def gaussian_shape(x, x0=0.25, sigma=0.08):
    return np.exp(-0.5*((x-x0)/sigma)**2)

def simulate(A, B, K, x0, dt=5e-3, T=5.0):
    steps = int(T/dt)
    x = x0.copy()
    hist = np.zeros((steps+1, x.size))
    E = np.zeros(steps+1)
    hist[0] = x
    E[0] = 0.5 * float(x.T @ x)
    for k in range(steps):
        u = -float(K @ x)  # scalar input
        xdot = A @ x + B.flatten() * u
        x = x + dt * xdot
        hist[k+1] = x
        E[k+1] = 0.5 * float(x.T @ x)
    return hist, E

def main():
    # PDE: x_t = alpha x_xx + b(x) u(t),  x(0,t)=x(L,t)=0
    alpha = 0.12
    L = 1.0
    N = 60

    D2, grid, dx = laplacian_1d_dirichlet(N, L)
    A = alpha * D2

    # Distributed actuator shape b(x) (bounded input operator in the PDE idealization)
    b = gaussian_shape(grid, x0=0.25, sigma=0.07)
    b = b / np.linalg.norm(b)  # normalize
    B = b.reshape(-1, 1)

    # Finite-dimensional LQR: minimize \int (x^T Q x + u^T R u) dt
    Q = np.eye(N)
    R = np.array([[2e-3]])

    # Continuous-time algebraic Riccati equation: A^T P + P A - P B R^{-1} B^T P + Q = 0
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)  # 1xN

    # Initial condition: a smooth bump
    x0 = np.exp(-80.0*(grid-0.75)**2)

    hist, E = simulate(A, B, K, x0, dt=2e-3, T=4.0)

    print("Grid points N =", N, "dx =", dx)
    print("Closed-loop gain K shape:", K.shape)
    print("Energy E(0)   =", E[0])
    print("Energy E(end) =", E[-1])
    print("Energy decay factor =", E[-1]/E[0])

if __name__ == "__main__":
    main()
