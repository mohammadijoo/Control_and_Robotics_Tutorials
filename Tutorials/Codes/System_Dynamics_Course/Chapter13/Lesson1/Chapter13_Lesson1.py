# Chapter13_Lesson1.py
"""System Dynamics — Chapter 13, Lesson 1
Modeling Two- and Multi-Degree-of-Freedom Mechanical Systems.

This script:
1) Builds M, C, K for a 2-DOF mass–spring–damper system,
2) Converts to first-order state-space form,
3) Simulates free and forced responses using SciPy's solve_ivp.

Dependencies:
  pip install numpy scipy matplotlib
"""

from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike
from dataclasses import dataclass
from typing import Callable, Tuple
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


@dataclass(frozen=True)
class TwoDOFParams:
    m1: float = 1.0
    m2: float = 0.8
    k1: float = 200.0  # spring to ground on mass 1
    k2: float = 150.0  # spring between masses
    k3: float = 100.0  # spring to ground on mass 2
    c1: float = 1.5    # damper to ground on mass 1
    c2: float = 1.0    # damper between masses
    c3: float = 0.8    # damper to ground on mass 2


def two_dof_mck(p: TwoDOFParams) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (M, C, K) for the standard 2-DOF chain."""
    M = np.diag([p.m1, p.m2])
    C = np.array([[p.c1 + p.c2, -p.c2],
                  [-p.c2, p.c2 + p.c3]], dtype=float)
    K = np.array([[p.k1 + p.k2, -p.k2],
                  [-p.k2, p.k2 + p.k3]], dtype=float)
    return M, C, K


def state_space_from_mck(M: np.ndarray, C: np.ndarray, K: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Build continuous-time state-space matrices for
        M qdd + C qd + K q = f(t).

    State x = [q; qd] ∈ R^{2n}.
    Then xdot = A x + B f, with f ∈ R^{n}.
    """
    n = M.shape[0]
    Minv = np.linalg.solve(M, np.eye(n))
    Z = np.zeros((n, n))
    I = np.eye(n)

    A = np.block([
        [Z, I],
        [-Minv @ K, -Minv @ C]
    ])
    B = np.vstack([Z, Minv])
    return A, B


def simulate(
    A: np.ndarray,
    B: np.ndarray,
    force: Callable[[float], np.ndarray],
    t_span: Tuple[float, float],
    x0: ArrayLike,
    t_eval: np.ndarray | None = None,
) -> solve_ivp:
    """Simulate xdot = A x + B f(t)."""
    x0 = np.asarray(x0, dtype=float).reshape(-1)

    def rhs(t: float, x: np.ndarray) -> np.ndarray:
        return A @ x + B @ force(t)

    sol = solve_ivp(rhs, t_span, x0, t_eval=t_eval, rtol=1e-8, atol=1e-10)
    if not sol.success:
        raise RuntimeError(sol.message)
    return sol


def main() -> None:
    p = TwoDOFParams()
    M, C, K = two_dof_mck(p)
    A, B = state_space_from_mck(M, C, K)

    # ---- Free response (f(t)=0) ----
    def f_free(t: float) -> np.ndarray:
        return np.zeros(2)

    t = np.linspace(0.0, 8.0, 2001)
    x0 = np.array([0.02, -0.01, 0.0, 0.0])  # [q1,q2,q1dot,q2dot]
    sol_free = simulate(A, B, f_free, (t[0], t[-1]), x0, t_eval=t)

    q1, q2 = sol_free.y[0], sol_free.y[1]

    # ---- Forced response (harmonic force on mass 1) ----
    F0 = 5.0
    w = 12.0  # rad/s
    def f_forced(t: float) -> np.ndarray:
        return np.array([F0 * np.cos(w * t), 0.0])

    sol_forced = simulate(A, B, f_forced, (t[0], t[-1]), x0, t_eval=t)
    q1f, q2f = sol_forced.y[0], sol_forced.y[1]

    # ---- Plot ----
    plt.figure()
    plt.plot(t, q1, label="q1 free")
    plt.plot(t, q2, label="q2 free")
    plt.xlabel("t [s]")
    plt.ylabel("displacement [m]")
    plt.grid(True)
    plt.legend()

    plt.figure()
    plt.plot(t, q1f, label="q1 forced")
    plt.plot(t, q2f, label="q2 forced")
    plt.xlabel("t [s]")
    plt.ylabel("displacement [m]")
    plt.grid(True)
    plt.legend()

    plt.show()


if __name__ == "__main__":
    main()
