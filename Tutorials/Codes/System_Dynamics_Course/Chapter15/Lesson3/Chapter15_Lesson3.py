"""
Chapter 15, Lesson 3 (System Dynamics)
Stiff systems and implicit methods: compare explicit and implicit solvers.
"""
import numpy as np
from scipy.integrate import solve_ivp
from time import perf_counter

def robertson(t, y):
    y1, y2, y3 = y
    return [
        -0.04*y1 + 1.0e4*y2*y3,
        0.04*y1 - 1.0e4*y2*y3 - 3.0e7*y2*y2,
        3.0e7*y2*y2
    ]

# Compare a nonstiff-oriented method vs stiff methods on a classic stiff IVP
t_span = (0.0, 1.0e2)
y0 = [1.0, 0.0, 0.0]

methods = ["RK45", "Radau", "BDF"]
for method in methods:
    t0 = perf_counter()
    sol = solve_ivp(robertson, t_span, y0, method=method, rtol=1e-6, atol=1e-10)
    dt = perf_counter() - t0
    njev = getattr(sol, "njev", 0)
    print(f"Method={method:>5s} | success={sol.success} | nfev={sol.nfev:6d} | njev={njev:4d} | time={dt:.4f}s")
    print("  y(tf) =", sol.y[:, -1])

print("\n--- Backward Euler on scalar stiff test equation y' = lambda*y ---")
lam = -1000.0
h = 0.05
N = 10
y = 1.0
t = 0.0
for n in range(N):
    # For y' = lam*y, backward Euler gives y_{n+1} = y_n / (1 - h*lam)
    y = y / (1.0 - h*lam)
    t += h
    print(f"n={n+1:2d}, t={t:.3f}, y={y:.6e}")
