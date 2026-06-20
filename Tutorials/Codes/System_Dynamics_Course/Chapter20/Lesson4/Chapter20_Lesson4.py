# Chapter20_Lesson4.py
"""
Chapter 20 — Chaos, Complex Dynamics, and Computational Tools
Lesson 4 — Computational Tools: MATLAB/Simulink, Python (SciPy), and Modelica for System Dynamics

This script demonstrates:
1) Continuous-time ODE simulation (Lorenz system) with SciPy solve_ivp (adaptive RK and stiff solvers)
2) Event detection for Poincaré sections
3) Largest Lyapunov exponent estimation (Benettin-style) for the Lorenz flow (intro level)
4) Discrete-time map simulation (logistic map), bifurcation data, and Lyapunov exponent of the map
5) Reproducible export to CSV for post-processing / plotting in any tool

Dependencies:
- numpy
- scipy
"""

from __future__ import annotations
import math
import csv
from dataclasses import dataclass
from typing import Callable, Tuple

import numpy as np
from scipy.integrate import solve_ivp


# ----------------------------
# 1) Lorenz system (ODE)
# ----------------------------
@dataclass
class LorenzParams:
    sigma: float = 10.0
    rho: float = 28.0
    beta: float = 8.0 / 3.0


def lorenz_rhs(t: float, x: np.ndarray, p: LorenzParams) -> np.ndarray:
    """dx/dt = f(x)."""
    X, Y, Z = x
    return np.array([
        p.sigma * (Y - X),
        X * (p.rho - Z) - Y,
        X * Y - p.beta * Z
    ], dtype=float)


def lorenz_jacobian(x: np.ndarray, p: LorenzParams) -> np.ndarray:
    """Jacobian J = df/dx."""
    X, Y, Z = x
    return np.array([
        [-p.sigma, p.sigma, 0.0],
        [p.rho - Z, -1.0, -X],
        [Y, X, -p.beta]
    ], dtype=float)


def event_poincare_x0(t: float, x: np.ndarray, p: LorenzParams) -> float:
    """Event: x crosses 0 with positive direction."""
    return x[0]


event_poincare_x0.terminal = False
event_poincare_x0.direction = 1.0


def simulate_lorenz(
    x0: np.ndarray,
    t_span: Tuple[float, float],
    p: LorenzParams,
    method: str = "RK45",
    rtol: float = 1e-9,
    atol: float = 1e-12,
    max_step: float = 0.05,
) -> solve_ivp:
    """Adaptive integration with optional event handling."""
    fun = lambda t, x: lorenz_rhs(t, x, p)
    ev = lambda t, x: event_poincare_x0(t, x, p)
    sol = solve_ivp(
        fun=fun,
        t_span=t_span,
        y0=x0,
        method=method,
        rtol=rtol,
        atol=atol,
        max_step=max_step,
        events=ev,
        dense_output=True
    )
    if not sol.success:
        raise RuntimeError(f"Integration failed: {sol.message}")
    return sol


def export_trajectory_csv(path: str, t: np.ndarray, y: np.ndarray) -> None:
    """Write trajectory to CSV: t,x,y,z."""
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z"])
        for i in range(t.size):
            w.writerow([float(t[i]), float(y[0, i]), float(y[1, i]), float(y[2, i])])


def export_points_csv(path: str, pts: np.ndarray, header=("x", "y", "z")) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(list(header))
        for row in pts:
            w.writerow([float(v) for v in row])


# ----------------------------
# 2) Largest Lyapunov exponent (Lorenz, intro level)
#    Benettin-style: integrate tangent dynamics and renormalize.
# ----------------------------
def lce_lorenz_benettin(
    x0: np.ndarray,
    p: LorenzParams,
    t_transient: float = 5.0,
    t_total: float = 50.0,
    dt_renorm: float = 0.05,
    method: str = "RK45",
) -> float:
    """
    Estimate largest Lyapunov exponent λ_max.

    We integrate the extended system:
      x' = f(x)
      v' = J(x) v
    and periodically renormalize v to avoid overflow.

    Notes:
    - This is an educational implementation; for research-grade estimation,
      use multiple tangent vectors and QR orthonormalization.
    """
    # Transient integration to approach attractor
    sol_tr = simulate_lorenz(x0, (0.0, t_transient), p, method=method, max_step=dt_renorm)
    x = sol_tr.y[:, -1].astype(float)

    # Initial tangent vector
    v = np.array([1.0, 0.0, 0.0], dtype=float)
    v /= np.linalg.norm(v)

    # Integrate in chunks of dt_renorm
    t = 0.0
    sum_log = 0.0
    n = 0

    def ext_rhs(t_local: float, z: np.ndarray) -> np.ndarray:
        x_local = z[0:3]
        v_local = z[3:6]
        dx = lorenz_rhs(t_local, x_local, p)
        J = lorenz_jacobian(x_local, p)
        dv = J @ v_local
        return np.concatenate([dx, dv])

    while t < t_total:
        z0 = np.concatenate([x, v])
        sol = solve_ivp(
            fun=ext_rhs,
            t_span=(0.0, dt_renorm),
            y0=z0,
            method=method,
            rtol=1e-9,
            atol=1e-12,
            max_step=dt_renorm
        )
        if not sol.success:
            raise RuntimeError(f"Extended integration failed: {sol.message}")
        z1 = sol.y[:, -1]
        x = z1[0:3]
        v = z1[3:6]
        norm_v = float(np.linalg.norm(v))
        if norm_v == 0.0:
            # extremely unlikely; re-seed
            v = np.array([1.0, 0.0, 0.0], dtype=float)
            norm_v = 1.0
        sum_log += math.log(norm_v)
        n += 1
        v = v / norm_v
        t += dt_renorm

    return sum_log / (n * dt_renorm)


# ----------------------------
# 3) Logistic map (discrete-time)
# ----------------------------
def logistic_map(r: float, x0: float, n: int) -> np.ndarray:
    """x_{k+1} = r x_k (1 - x_k), returns array length n+1."""
    xs = np.empty(n + 1, dtype=float)
    x = float(x0)
    xs[0] = x
    for k in range(n):
        x = r * x * (1.0 - x)
        xs[k + 1] = x
    return xs


def logistic_bifurcation_data(
    r_values: np.ndarray,
    x0: float = 0.2,
    n_transient: int = 1000,
    n_keep: int = 200,
) -> np.ndarray:
    """
    Return (r, x) pairs after discarding transient.
    Useful for bifurcation diagrams.
    """
    rows = []
    for r in r_values:
        xs = logistic_map(float(r), x0, n_transient + n_keep)
        tail = xs[-n_keep:]
        for x in tail:
            rows.append((float(r), float(x)))
    return np.array(rows, dtype=float)


def logistic_lyapunov(r: float, x0: float = 0.2, n_transient: int = 1000, n: int = 5000) -> float:
    """
    Largest Lyapunov exponent for logistic map:
      λ = lim (1/N) Σ ln | f'(x_k) |, with f'(x)=r(1-2x)
    """
    xs = logistic_map(r, x0, n_transient + n)
    xs = xs[n_transient:]  # drop transient
    deriv = np.abs(r * (1.0 - 2.0 * xs))
    # Avoid log(0)
    deriv = np.maximum(deriv, 1e-300)
    return float(np.mean(np.log(deriv)))


# ----------------------------
# 4) Main demo
# ----------------------------
def main() -> None:
    out_dir = "outputs_ch20_l4"
    os.makedirs(out_dir, exist_ok=True)

    p = LorenzParams(sigma=10.0, rho=28.0, beta=8.0/3.0)
    x0 = np.array([1.0, 1.0, 1.0], dtype=float)

    # Lorenz simulation with adaptive RK (Dormand–Prince-like)
    sol = simulate_lorenz(
        x0=x0,
        t_span=(0.0, 40.0),
        p=p,
        method="RK45",
        max_step=0.02
    )
    export_trajectory_csv(os.path.join(out_dir, "lorenz_traj.csv"), sol.t, sol.y)

    # Poincaré section points (events at x=0 crossing)
    if sol.t_events and len(sol.t_events[0]) > 0:
        t_ev = sol.t_events[0]
        y_ev = sol.sol(t_ev) if sol.sol is not None else None
        if y_ev is not None:
            pts = y_ev.T  # rows
            export_points_csv(os.path.join(out_dir, "lorenz_poincare_x0.csv"), pts, header=("x", "y", "z"))

    # Largest Lyapunov exponent (educational)
    lce = lce_lorenz_benettin(x0=x0, p=p, t_transient=5.0, t_total=60.0, dt_renorm=0.05)
    with open(os.path.join(out_dir, "lorenz_lce.txt"), "w") as f:
        f.write(f"lambda_max ~ {lce:.6f}\n")

    # Logistic map: bifurcation data and Lyapunov
    r_vals = np.linspace(2.5, 4.0, 400)
    bif = logistic_bifurcation_data(r_vals, x0=0.2, n_transient=800, n_keep=120)
    np.savetxt(os.path.join(out_dir, "logistic_bifurcation.csv"), bif, delimiter=",", header="r,x", comments="")

    lyap_rows = []
    for r in np.linspace(2.8, 4.0, 200):
        lyap_rows.append((float(r), logistic_lyapunov(float(r), x0=0.2, n_transient=1000, n=4000)))
    np.savetxt(os.path.join(out_dir, "logistic_lyapunov.csv"), np.array(lyap_rows), delimiter=",", header="r,lambda", comments="")

    print("Done. Outputs written to:", out_dir)
    print("Lorenz LCE estimate:", lce)


if __name__ == "__main__":
    import os
    main()
