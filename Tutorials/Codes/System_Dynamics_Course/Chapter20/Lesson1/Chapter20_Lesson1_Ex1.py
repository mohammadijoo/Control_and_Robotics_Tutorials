# Chapter20_Lesson1_Ex1.py
"""
Exercise 1 — Logistic map sensitivity and invariant interval.

Task:
- Pick r in (0,4], choose two nearby initial conditions in [0,1], iterate.
- Plot |x_n - y_n| over n on semilog scale.
- Verify invariance of [0,1] for r in (0,4].

This file is intentionally short for students to extend.
"""

import numpy as np
import matplotlib.pyplot as plt


def logistic(r, x):
    return r * x * (1.0 - x)


def iterate(r, x0, n):
    x = np.empty(n + 1)
    x[0] = x0
    for k in range(n):
        x[k + 1] = logistic(r, x[k])
    return x


if __name__ == "__main__":
    r = 3.9
    n = 200
    x0 = 0.2000000000
    y0 = 0.2000000001

    x = iterate(r, x0, n)
    y = iterate(r, y0, n)
    d = np.abs(x - y)

    plt.figure()
    plt.semilogy(np.arange(n + 1), d + 1e-30)
    plt.xlabel("n")
    plt.ylabel("|x_n - y_n|")
    plt.title("Log-distance between nearby logistic-map trajectories")
    plt.grid(True)
    plt.show()
