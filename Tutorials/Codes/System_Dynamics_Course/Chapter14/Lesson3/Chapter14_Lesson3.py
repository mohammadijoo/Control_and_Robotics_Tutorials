"""
Chapter 14 - Nonlinear System Dynamics
Lesson 3 - Linearization vs. True Nonlinear Behavior: When Linear Models Fail

File: Chapter14_Lesson3.py
Dependencies: numpy, matplotlib (optional), scipy (optional)
This script:
  1) Computes Jacobian linearizations at equilibria
  2) Simulates nonlinear and linearized models (RK4 from scratch)
  3) Demonstrates cases where linearization is conclusive / inconclusive / misleading globally
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Callable, List, Tuple

import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception:
    HAS_MPL = False


Vector = np.ndarray
Func = Callable[[float, Vector], Vector]


def rk4(f: Func, t0: float, tf: float, x0: Vector, h: float) -> Tuple[np.ndarray, np.ndarray]:
    """Classic fixed-step RK4 integrator (from scratch)."""
    n_steps = int(math.ceil((tf - t0) / h))
    ts = np.zeros(n_steps + 1, dtype=float)
    xs = np.zeros((n_steps + 1, x0.size), dtype=float)
    ts[0] = t0
    xs[0] = x0.copy()
    t = t0
    x = x0.copy()
    for k in range(n_steps):
        if t + h > tf:
            h = tf - t
        k1 = f(t, x)
        k2 = f(t + 0.5 * h, x + 0.5 * h * k1)
        k3 = f(t + 0.5 * h, x + 0.5 * h * k2)
        k4 = f(t + h, x + h * k3)
        x = x + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        t = t + h
        ts[k + 1] = t
        xs[k + 1] = x
    return ts, xs


def jacobian_fd(g: Callable[[Vector], Vector], x: Vector, eps: float = 1e-6) -> np.ndarray:
    """Finite-difference Jacobian J = dg/dx at x (central differences)."""
    x = np.asarray(x, dtype=float)
    n = x.size
    J = np.zeros((n, n), dtype=float)
    for i in range(n):
        dx = np.zeros(n, dtype=float)
        dx[i] = eps
        gp = g(x + dx)
        gm = g(x - dx)
        J[:, i] = (gp - gm) / (2.0 * eps)
    return J


# ---------------------------
# Example A: Nonhyperbolic equilibrium (linearization inconclusive)
# xdot = -x^3  (equilibrium at 0 is asymptotically stable)
# ---------------------------

def fA(t: float, x: Vector) -> Vector:
    return np.array([-x[0] ** 3], dtype=float)

def linA_at0(t: float, dx: Vector) -> Vector:
    # Jacobian A = 0 at x*=0
    return np.array([0.0], dtype=float)


# ---------------------------
# Example B: Linear center vs nonlinear spiral (linearization qualitatively wrong)
# xdot = -y + x (x^2 + y^2)
# ydot =  x + y (x^2 + y^2)
# At origin: A = [[0,-1],[1,0]] (pure rotation, center)
# Nonlinear term makes rdot = r^3 (spiral out)
# ---------------------------

def fB(t: float, z: Vector) -> Vector:
    x, y = z[0], z[1]
    r2 = x * x + y * y
    return np.array([-y + x * r2, x + y * r2], dtype=float)

A_B = np.array([[0.0, -1.0],
                [1.0,  0.0]], dtype=float)

def linB_at0(t: float, dz: Vector) -> Vector:
    return A_B @ dz


# ---------------------------
# Example C: Locally stable but not globally stable
# xdot = -x + x^3  (equilibria at -1,0,1; 0 locally stable, ROA is |x0|<1)
# ---------------------------

def fC(t: float, x: Vector) -> Vector:
    return np.array([-x[0] + x[0] ** 3], dtype=float)

def linC_at0(t: float, dx: Vector) -> Vector:
    # Jacobian A = -1 at x*=0
    return np.array([-dx[0]], dtype=float)


@dataclass
class SimResult:
    t: np.ndarray
    x_nl: np.ndarray
    x_lin: np.ndarray


def simulate_compare(f_nl: Func, f_lin: Func, xeq: Vector, x0: Vector,
                     t0: float = 0.0, tf: float = 20.0, h: float = 1e-3) -> SimResult:
    # Linear model is in perturbation coordinates: d = x - xeq
    d0 = x0 - xeq
    t_nl, x_nl = rk4(f_nl, t0, tf, x0, h)
    t_lin, d_lin = rk4(f_lin, t0, tf, d0, h)
    x_lin = d_lin + xeq.reshape(1, -1)
    # Ensure same time grid (RK4 uses same grid by construction)
    return SimResult(t=t_nl, x_nl=x_nl, x_lin=x_lin)


def main() -> None:
    np.set_printoptions(precision=4, suppress=True)

    # ----- Example A
    print("\nExample A: xdot = -x^3 (nonhyperbolic at 0, but stable)")
    xeq = np.array([0.0])
    x0 = np.array([0.8])
    resA = simulate_compare(fA, linA_at0, xeq, x0, tf=10.0, h=1e-3)
    print("x(T) nonlinear:", resA.x_nl[-1, 0], "  |  x(T) linear:", resA.x_lin[-1, 0])

    # ----- Example B
    print("\nExample B: linear center vs nonlinear spiral-out")
    z0 = np.array([0.2, 0.0])
    zeq = np.array([0.0, 0.0])
    resB = simulate_compare(fB, linB_at0, zeq, z0, tf=25.0, h=1e-3)
    r_nl = np.sqrt(resB.x_nl[:, 0] ** 2 + resB.x_nl[:, 1] ** 2)
    r_lin = np.sqrt(resB.x_lin[:, 0] ** 2 + resB.x_lin[:, 1] ** 2)
    print("r(0) nonlinear:", r_nl[0], " r(T) nonlinear:", r_nl[-1])
    print("r(0) linear   :", r_lin[0], " r(T) linear   :", r_lin[-1])

    # ----- Example C
    print("\nExample C: local vs global mismatch (region of attraction issue)")
    xeq = np.array([0.0])
    for x_init in [0.2, 0.9, 1.1, 1.5]:
        resC = simulate_compare(fC, linC_at0, xeq, np.array([x_init]), tf=10.0, h=1e-3)
        print(f"x0={x_init:.2f} -> x(T) nonlinear={resC.x_nl[-1,0]:.4f} | linear={resC.x_lin[-1,0]:.4f}")

    # ----- Jacobian verification (finite difference)
    print("\nFinite-difference Jacobian checks:")
    JB = jacobian_fd(lambda z: fB(0.0, z), np.array([0.0, 0.0]))
    print("J_B(0) =\n", JB)
    eigB = np.linalg.eigvals(JB)
    print("eig(J_B(0)) =", eigB)

    JC0 = jacobian_fd(lambda x: fC(0.0, x), np.array([0.0]))
    print("J_C(0) =", JC0[0,0], " expected -1")

    # ----- Optional plots
    if HAS_MPL:
        # Phase-plane comparison for Example B
        plt.figure()
        plt.plot(resB.x_lin[:, 0], resB.x_lin[:, 1], label="Linearized (center)")
        plt.plot(resB.x_nl[:, 0], resB.x_nl[:, 1], label="Nonlinear (spiral out)")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("Example B: Phase portrait (linear vs nonlinear)")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.savefig("Chapter14_Lesson3_ExampleB_Phase.png", dpi=150)

        # Radius vs time for Example B
        plt.figure()
        plt.plot(resB.t, r_lin, label="r(t) linear")
        plt.plot(resB.t, r_nl, label="r(t) nonlinear")
        plt.xlabel("t")
        plt.ylabel("r")
        plt.title("Example B: Radius growth (linear vs nonlinear)")
        plt.grid(True)
        plt.legend()
        plt.savefig("Chapter14_Lesson3_ExampleB_Radius.png", dpi=150)

        print("\nSaved plots: Chapter14_Lesson3_ExampleB_Phase.png, Chapter14_Lesson3_ExampleB_Radius.png")
    else:
        print("\nmatplotlib not available; skipping plots.")

    # Save CSV for Example B
    out = np.column_stack([resB.t, resB.x_lin[:,0], resB.x_lin[:,1], resB.x_nl[:,0], resB.x_nl[:,1]])
    np.savetxt("Chapter14_Lesson3_ExampleB.csv", out, delimiter=",",
               header="t,x_lin,y_lin,x_nl,y_nl", comments="")
    print("Saved CSV: Chapter14_Lesson3_ExampleB.csv")


if __name__ == "__main__":
    main()
