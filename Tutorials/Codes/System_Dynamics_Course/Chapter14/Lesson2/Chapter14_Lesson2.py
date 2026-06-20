"""
Chapter14_Lesson2.py
Equilibrium points, phase portraits, and trajectories for planar (2D) autonomous systems.

Requires:
  numpy, scipy, matplotlib
Install (example):
  pip install numpy scipy matplotlib
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from typing import Callable, List, Tuple, Dict

from scipy.integrate import solve_ivp
from scipy.optimize import root
import matplotlib.pyplot as plt


# -----------------------------
# 1) Define a 2D autonomous system
# -----------------------------
@dataclass(frozen=True)
class PlanarSystem:
    """Planar autonomous system: z' = F(z) with z=(x,y)."""
    a: float = 1.0  # parameter for example system

    def f(self, x: float, y: float) -> float:
        # dx/dt
        return x - x**3 - y

    def g(self, x: float, y: float) -> float:
        # dy/dt
        return x + self.a * y

    def F(self, t: float, z: np.ndarray) -> np.ndarray:
        x, y = float(z[0]), float(z[1])
        return np.array([self.f(x, y), self.g(x, y)], dtype=float)


# -----------------------------
# 2) Equilibria and Jacobian
# -----------------------------
def jacobian_fd(sys: PlanarSystem, z: np.ndarray, h: float = 1e-6) -> np.ndarray:
    """Finite-difference Jacobian J = dF/dz at z (2x2)."""
    z = np.asarray(z, dtype=float)
    J = np.zeros((2, 2), dtype=float)
    f0 = sys.F(0.0, z)
    for i in range(2):
        zp = z.copy()
        zp[i] += h
        fp = sys.F(0.0, zp)
        J[:, i] = (fp - f0) / h
    return J


def find_equilibria(
    sys: PlanarSystem,
    guesses: List[Tuple[float, float]],
    tol: float = 1e-8
) -> List[np.ndarray]:
    """
    Find equilibria by running a nonlinear root solver from multiple initial guesses,
    then deduplicating by tolerance.
    """
    sols: List[np.ndarray] = []

    def fun(z):
        return sys.F(0.0, np.asarray(z, dtype=float))

    for gx, gy in guesses:
        res = root(fun, x0=np.array([gx, gy], dtype=float), method="hybr")
        if not res.success:
            continue
        z = res.x.astype(float)
        # Deduplicate
        if all(np.linalg.norm(z - s) > tol for s in sols):
            sols.append(z)
    return sols


# -----------------------------
# 3) Local classification via trace/determinant and eigenvalues
# -----------------------------
@dataclass(frozen=True)
class LinClass:
    kind: str
    trace: float
    det: float
    disc: float
    eig: Tuple[complex, complex]


def classify_2x2(J: np.ndarray, eps: float = 1e-10) -> LinClass:
    """
    Classify planar linearization z' = J z at an equilibrium using invariants:
      tr = trace(J), det = det(J), disc = tr^2 - 4 det.
    """
    tr = float(np.trace(J))
    det = float(np.linalg.det(J))
    disc = float(tr * tr - 4.0 * det)
    eigvals = np.linalg.eigvals(J)
    lam1, lam2 = complex(eigvals[0]), complex(eigvals[1])

    # Robust comparisons with eps
    if det < -eps:
        kind = "saddle (hyperbolic)"
    elif abs(det) <= eps:
        kind = "non-isolated or degenerate (det ~ 0)"
    else:
        # det > 0
        if disc > eps:
            # real eigenvalues
            if tr < -eps:
                kind = "stable node"
            elif tr > eps:
                kind = "unstable node"
            else:
                kind = "improper / star node (tr ~ 0, disc > 0)"
        elif disc < -eps:
            # complex conjugate
            if tr < -eps:
                kind = "stable spiral (focus)"
            elif tr > eps:
                kind = "unstable spiral (focus)"
            else:
                kind = "center (linear); nonlinear terms decide"
        else:
            # disc ~ 0: repeated eigenvalue
            if tr < -eps:
                kind = "stable degenerate node"
            elif tr > eps:
                kind = "unstable degenerate node"
            else:
                kind = "center/degenerate (tr ~ 0, disc ~ 0)"
    return LinClass(kind=kind, trace=tr, det=det, disc=disc, eig=(lam1, lam2))


# -----------------------------
# 4) Phase portrait plotting + trajectories
# -----------------------------
def plot_phase_portrait(
    sys: PlanarSystem,
    equilibria: List[np.ndarray],
    xlim=(-2.5, 2.5),
    ylim=(-2.5, 2.5),
    ngrid: int = 25,
    tmax: float = 12.0,
    initials: List[Tuple[float, float]] | None = None,
    title: str = "Phase portrait"
) -> None:
    if initials is None:
        initials = [(-2, -2), (-2, 0), (-2, 2), (0.5, -2), (0.5, 2), (2, -2), (2, 0), (2, 2)]

    # Vector field grid
    xs = np.linspace(xlim[0], xlim[1], ngrid)
    ys = np.linspace(ylim[0], ylim[1], ngrid)
    X, Y = np.meshgrid(xs, ys)
    U = np.zeros_like(X)
    V = np.zeros_like(Y)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            U[i, j] = sys.f(X[i, j], Y[i, j])
            V[i, j] = sys.g(X[i, j], Y[i, j])

    plt.figure()
    plt.streamplot(X, Y, U, V, density=1.0)

    # Plot equilibria
    for z in equilibria:
        plt.plot([z[0]], [z[1]], marker="o")
        J = jacobian_fd(sys, z)
        cls = classify_2x2(J)
        plt.text(z[0] + 0.05, z[1] + 0.05, cls.kind, fontsize=8)

    # Plot trajectories
    def rhs(t, z):
        return sys.F(t, z)

    for x0, y0 in initials:
        sol = solve_ivp(rhs, (0.0, tmax), y0=np.array([x0, y0], dtype=float),
                        rtol=1e-7, atol=1e-9, max_step=0.05)
        plt.plot(sol.y[0], sol.y[1], linewidth=1.0)

    plt.xlim(*xlim)
    plt.ylim(*ylim)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def main() -> None:
    sys = PlanarSystem(a=1.0)

    # Try a grid of initial guesses to find equilibria
    guesses = []
    for gx in np.linspace(-2.0, 2.0, 9):
        for gy in np.linspace(-2.0, 2.0, 9):
            guesses.append((float(gx), float(gy)))

    eqs = find_equilibria(sys, guesses)
    print("Equilibria (approx):")
    for z in eqs:
        J = jacobian_fd(sys, z)
        cls = classify_2x2(J)
        print(f"  z* = [{z[0]: .6f}, {z[1]: .6f}]  trace={cls.trace: .6f} det={cls.det: .6f} -> {cls.kind}")

    plot_phase_portrait(sys, eqs, title="Example planar nonlinear system: x' = x - x^3 - y,  y' = x + a y")


if __name__ == "__main__":
    main()
