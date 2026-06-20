"""
System Dynamics — Chapter 14 (Nonlinear System Dynamics)
Lesson 1: Sources and Types of Nonlinearities in Engineering Systems

This script demonstrates a single-DOF nonlinear mass–spring–damper with:
- Cubic stiffness (geometric/material nonlinearity proxy)
- Coulomb + viscous friction (dissipative nonlinearity, smoothed)
- Actuator saturation and optional sensor dead-zone (static, memoryless nonlinearities)

Outputs: plots and a CSV trajectory for cross-language comparison.
"""

from __future__ import annotations

import math
import csv
from dataclasses import dataclass

import numpy as np

# Optional SciPy; the script falls back to an explicit RK4 if SciPy is unavailable.
try:
    from scipy.integrate import solve_ivp
    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False

import matplotlib.pyplot as plt


def sat(u: float, umax: float) -> float:
    """Symmetric saturation."""
    return float(np.clip(u, -umax, umax))


def deadzone(x: float, d: float) -> float:
    """
    Symmetric dead-zone:
        y = 0                 if |x| < d
        y = x - d*sign(x)     otherwise
    """
    ax = abs(x)
    if ax < d:
        return 0.0
    return x - d * math.copysign(1.0, x)


def coulomb_friction(v: float, Fc: float, vs: float) -> float:
    """
    Smoothed Coulomb friction ~ Fc*sign(v), using tanh(v/vs) to avoid discontinuity at v=0.
    Smaller vs -> sharper transition.
    """
    return Fc * math.tanh(v / vs)


@dataclass
class Params:
    m: float = 1.0       # kg
    c: float = 0.4       # N*s/m
    k: float = 4.0       # N/m
    k3: float = 8.0      # N/m^3 (cubic stiffness)
    Fc: float = 0.8      # N (Coulomb level)
    vs: float = 0.02     # m/s (smoothing velocity)
    b: float = 1.0       # N per unit input
    umax: float = 1.5    # saturation limit (input units)
    dz: float = 0.0      # sensor dead-zone (meters); set >0 to enable


def input_u(t: float) -> float:
    """
    Excitation signal (can be replaced by any input profile):
    a combination of low-frequency sine + small bias.
    """
    return 1.2 * math.sin(1.0 * t) + 0.3 * math.sin(3.0 * t)


def dynamics(t: float, x: np.ndarray, p: Params) -> np.ndarray:
    """
    State x = [position, velocity].
    Model:
        m xdd + c xd + k x + k3 x^3 + Fc*tanh(xd/vs) = b*sat(u(t), umax)
    """
    pos, vel = float(x[0]), float(x[1])

    # Optional sensor dead-zone (used only to illustrate a measurement nonlinearity)
    pos_meas = deadzone(pos, p.dz) if p.dz > 0 else pos

    u = input_u(t)
    u_sat = sat(u, p.umax)

    spring = p.k * pos + p.k3 * (pos ** 3)
    fric = p.c * vel + coulomb_friction(vel, p.Fc, p.vs)

    acc = (p.b * u_sat - spring - fric) / p.m
    return np.array([vel, acc], dtype=float)


def rk4(f, t0: float, tf: float, x0: np.ndarray, h: float, p: Params):
    """Fixed-step RK4 integrator (educational baseline)."""
    ts = [t0]
    xs = [x0.astype(float).copy()]
    t = t0
    x = x0.astype(float).copy()

    while t < tf - 1e-12:
        if t + h > tf:
            h = tf - t
        k1 = f(t, x, p)
        k2 = f(t + 0.5 * h, x + 0.5 * h * k1, p)
        k3 = f(t + 0.5 * h, x + 0.5 * h * k2, p)
        k4 = f(t + h, x + h * k3, p)
        x = x + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        t = t + h
        ts.append(t)
        xs.append(x.copy())

    return np.array(ts), np.vstack(xs)


def simulate(p: Params, t0: float = 0.0, tf: float = 20.0):
    x0 = np.array([0.4, 0.0], dtype=float)

    if _HAVE_SCIPY:
        sol = solve_ivp(
            fun=lambda t, x: dynamics(t, x, p),
            t_span=(t0, tf),
            y0=x0,
            max_step=0.01,
            rtol=1e-7,
            atol=1e-9,
        )
        t = sol.t
        x = sol.y.T
    else:
        t, x = rk4(dynamics, t0, tf, x0, h=0.005, p=p)

    return t, x


def export_csv(path: str, t: np.ndarray, x: np.ndarray):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "xdot"])
        for ti, (xi, vi) in zip(t, x):
            w.writerow([float(ti), float(xi), float(vi)])


def main():
    p_nl = Params()
    p_lin = Params(k3=0.0, Fc=0.0, umax=1e9)  # "linear-ish" comparison (no cubic, no Coulomb, no sat)

    t1, x1 = simulate(p_nl)
    t2, x2 = simulate(p_lin)

    export_csv("Chapter14_Lesson1_trace_nonlinear.csv", t1, x1)
    export_csv("Chapter14_Lesson1_trace_linear.csv", t2, x2)

    plt.figure()
    plt.plot(t1, x1[:, 0], label="x(t) nonlinear")
    plt.plot(t2, x2[:, 0], label="x(t) linear-ish", linestyle="--")
    plt.xlabel("t [s]")
    plt.ylabel("position x [m]")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(t1, x1[:, 1], label="xdot(t) nonlinear")
    plt.plot(t2, x2[:, 1], label="xdot(t) linear-ish", linestyle="--")
    plt.xlabel("t [s]")
    plt.ylabel("velocity xdot [m/s]")
    plt.legend()
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()
