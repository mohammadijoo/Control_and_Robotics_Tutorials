"""
Chapter 14 - Nonlinear System Dynamics
Lesson 3 - Exercise 1

File: Chapter14_Lesson3_Ex1.py
Goal: Empirically estimate a region of attraction for xdot = -x + x^3 around x*=0
by simulating many initial conditions and classifying convergence to 0.

Dependencies: numpy
Optional: matplotlib
"""

from __future__ import annotations
import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception:
    HAS_MPL = False


def rk4_scalar(f, t0, tf, x0, h):
    n_steps = int(np.ceil((tf - t0) / h))
    t = t0
    x = float(x0)
    xs = np.zeros(n_steps + 1)
    ts = np.zeros(n_steps + 1)
    xs[0] = x
    ts[0] = t
    for k in range(n_steps):
        if t + h > tf:
            h = tf - t
        k1 = f(t, x)
        k2 = f(t + 0.5*h, x + 0.5*h*k1)
        k3 = f(t + 0.5*h, x + 0.5*h*k2)
        k4 = f(t + h, x + h*k3)
        x = x + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        t = t + h
        xs[k+1] = x
        ts[k+1] = t
    return ts, xs


def f(t, x):
    return -x + x**3


def main():
    rng = np.random.default_rng(7)
    N = 400
    t0, tf, h = 0.0, 10.0, 1e-3
    eps = 1e-2

    x0s = rng.uniform(-1.6, 1.6, size=N)
    xT = np.zeros(N)
    converged = np.zeros(N, dtype=int)

    for i, x0 in enumerate(x0s):
        _, xs = rk4_scalar(f, t0, tf, x0, h)
        xT[i] = xs[-1]
        converged[i] = 1 if abs(xs[-1]) < eps else 0

    # Save results
    out = np.column_stack([x0s, xT, converged])
    np.savetxt("Chapter14_Lesson3_Ex1_ROA.csv", out, delimiter=",",
               header="x0,xT,converged_to_0", comments="")

    # Quick summary
    conv_rate = converged.mean()
    print("Convergence fraction (to |x(T)| < eps):", conv_rate)

    # Optional visualization
    if HAS_MPL:
        plt.figure()
        plt.scatter(x0s, xT, s=10)
        plt.xlabel("x0")
        plt.ylabel("x(T)")
        plt.title("Empirical convergence for xdot = -x + x^3")
        plt.grid(True)
        plt.savefig("Chapter14_Lesson3_Ex1_ROA.png", dpi=150)
        print("Saved: Chapter14_Lesson3_Ex1_ROA.png")

    print("Saved CSV: Chapter14_Lesson3_Ex1_ROA.csv")


if __name__ == "__main__":
    main()
