"""
Chapter19_Lesson3.py
Finite Difference (FD) and Finite Element (FE) Approximations (Conceptual Level)

This script provides:
  1) 1D heat equation discretization via finite differences (method of lines),
     with explicit FTCS and implicit theta-method (e.g., Crank–Nicolson).
  2) 1D Poisson / steady diffusion via linear finite elements (Galerkin FEM).

Dependencies:
  - numpy
  - scipy (sparse matrices and solvers)
  - matplotlib (optional, for plots)

Author: (course material generator)
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from typing import Callable, Tuple

from scipy.sparse import diags, csc_matrix, identity
from scipy.sparse.linalg import spsolve


@dataclass
class Grid1D:
    L: float = 1.0
    N: int = 101  # number of nodes including boundaries

    def __post_init__(self) -> None:
        if self.N < 3:
            raise ValueError("N must be >= 3.")
        self.x = np.linspace(0.0, self.L, self.N)
        self.dx = self.x[1] - self.x[0]


def laplacian_dirichlet_1d(N: int, dx: float) -> csc_matrix:
    """
    FD Laplacian on interior nodes with homogeneous Dirichlet boundaries.
    Unknowns are u[1:-1] (size N-2).
    """
    n = N - 2
    main = -2.0 * np.ones(n) / (dx * dx)
    off = 1.0 * np.ones(n - 1) / (dx * dx)
    A = diags([off, main, off], offsets=[-1, 0, 1], format="csc")
    return A


def heat_fd_explicit_ftcs(
    u0: np.ndarray,
    alpha: float,
    grid: Grid1D,
    dt: float,
    steps: int,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Solve u_t = alpha u_xx with FD in space and forward Euler in time (FTCS).
    Dirichlet boundaries are taken from u0[0] and u0[-1] and kept fixed.

    Stability (1D): r = alpha*dt/dx^2 <= 1/2  (classical sufficient condition).
    """
    u = u0.copy()
    dx = grid.dx
    r = alpha * dt / (dx * dx)
    if r > 0.5:
        print(f"[warning] FTCS may be unstable: r = {r:.4f} > 0.5")

    us = [u.copy()]
    ts = [0.0]
    for n in range(steps):
        un = u.copy()
        # interior update
        u[1:-1] = un[1:-1] + r * (un[2:] - 2.0 * un[1:-1] + un[:-2])
        us.append(u.copy())
        ts.append((n + 1) * dt)
    return np.array(ts), np.array(us)


def heat_fd_theta_method(
    u0: np.ndarray,
    alpha: float,
    grid: Grid1D,
    dt: float,
    steps: int,
    theta: float = 0.5,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Theta-method on the method-of-lines system:
        u_t = alpha * A u + b(t), with Dirichlet BC.

    In FD form for interior vector v = u[1:-1]:
        (I - theta*dt*alpha*A) v^{n+1} = (I + (1-theta)*dt*alpha*A) v^n + dt*rhs_bc

    theta = 0   -> forward Euler  (explicit)
    theta = 1/2 -> Crank–Nicolson (A-stable, 2nd order in time)
    theta = 1   -> backward Euler (A-stable, 1st order in time)
    """
    u = u0.copy()
    dx = grid.dx
    N = grid.N
    A = laplacian_dirichlet_1d(N, dx)

    n = N - 2
    I = identity(n, format="csc")
    LHS = (I - theta * dt * alpha * A)
    RHS = (I + (1.0 - theta) * dt * alpha * A)

    # fixed Dirichlet values
    uL = float(u0[0])
    uR = float(u0[-1])

    us = [u.copy()]
    ts = [0.0]

    for k in range(steps):
        v = u[1:-1].copy()

        # Boundary contribution for Laplacian: A*v assumes uL=uR=0,
        # so we add the missing terms.
        bc = np.zeros(n)
        bc[0] += uL / (dx * dx)
        bc[-1] += uR / (dx * dx)

        rhs = RHS @ v + dt * alpha * bc
        v_new = spsolve(LHS, rhs)
        u[1:-1] = v_new
        us.append(u.copy())
        ts.append((k + 1) * dt)

    return np.array(ts), np.array(us)


def fem_poisson_1d_linear(
    Nel: int,
    L: float,
    f: Callable[[np.ndarray], np.ndarray],
    u0: float = 0.0,
    uL: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Solve -u'' = f(x) on (0,L) with Dirichlet BC u(0)=u0, u(L)=uL
    using linear (P1) FEM on a uniform mesh with Nel elements.

    Weak form:
        find u in H^1_0 + lift such that ∫ u' v' dx = ∫ f v dx  for all v in H^1_0.

    Assembly uses element stiffness:
        Ke = (1/h) [[ 1, -1],
                    [-1,  1]]
    and load with midpoint quadrature on each element.
    """
    if Nel < 1:
        raise ValueError("Nel must be >= 1.")
    x = np.linspace(0.0, L, Nel + 1)
    h = x[1] - x[0]
    Nn = Nel + 1  # nodes

    K = np.zeros((Nn, Nn))
    F = np.zeros(Nn)

    for e in range(Nel):
        i, j = e, e + 1
        Ke = (1.0 / h) * np.array([[1.0, -1.0], [-1.0, 1.0]])
        xm = 0.5 * (x[i] + x[j])
        fe = f(np.array([xm]))[0]
        # consistent load for linear basis with midpoint quadrature:
        # ∫ f phi_i dx ≈ f(xm) * h/2 for each local node
        Fe = fe * (h / 2.0) * np.array([1.0, 1.0])

        K[i:i+2, i:i+2] += Ke
        F[i:i+2] += Fe

    # Dirichlet BC via elimination (simple for teaching)
    fixed = {0: u0, Nn - 1: uL}
    free = np.array([k for k in range(Nn) if k not in fixed])

    # modify RHS for known values
    for idx, val in fixed.items():
        F = F - K[:, idx] * val

    Kff = K[np.ix_(free, free)]
    Ff = F[free]
    uf = np.linalg.solve(Kff, Ff)

    u = np.zeros(Nn)
    u[free] = uf
    for idx, val in fixed.items():
        u[idx] = val
    return x, u


def main() -> None:
    # --- Heat equation example ---
    grid = Grid1D(L=1.0, N=101)
    alpha = 0.05

    # initial condition with Dirichlet boundaries fixed to 0
    u0 = np.sin(np.pi * grid.x)
    u0[0] = 0.0
    u0[-1] = 0.0

    # explicit (stable) and Crank–Nicolson
    dt_exp = 0.4 * (grid.dx * grid.dx) / alpha  # r=0.4 <= 0.5
    steps = 200

    t_exp, u_exp = heat_fd_explicit_ftcs(u0, alpha, grid, dt_exp, steps)
    t_cn, u_cn = heat_fd_theta_method(u0, alpha, grid, dt_exp, steps, theta=0.5)

    print("Heat FD done. Final time:", t_exp[-1])

    # --- Poisson FEM example ---
    f = lambda x: (np.pi ** 2) * np.sin(np.pi * x)  # so exact u = sin(pi x)
    x_fem, u_fem = fem_poisson_1d_linear(Nel=40, L=1.0, f=f, u0=0.0, uL=0.0)
    u_exact = np.sin(np.pi * x_fem)

    err_inf = np.max(np.abs(u_fem - u_exact))
    print("FEM Poisson max error:", err_inf)

    # Optional plot (comment out if running headless)
    try:
        import matplotlib.pyplot as plt

        plt.figure()
        plt.plot(grid.x, u_exp[-1], label="FD explicit (final)")
        plt.plot(grid.x, u_cn[-1], "--", label="FD Crank–Nicolson (final)")
        plt.legend()
        plt.xlabel("x")
        plt.ylabel("u")
        plt.title("1D Heat Equation: FD in Space")
        plt.show()

        plt.figure()
        plt.plot(x_fem, u_fem, "o-", label="FEM P1")
        plt.plot(x_fem, u_exact, "--", label="exact")
        plt.legend()
        plt.xlabel("x")
        plt.ylabel("u")
        plt.title("1D Poisson: Linear FEM")
        plt.show()
    except Exception as e:
        print("Plot skipped:", e)


if __name__ == "__main__":
    main()
