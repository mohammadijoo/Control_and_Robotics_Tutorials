# Chapter15_Lesson5.py
"""
Numerical Stability, Error Control, and Model Verification via Simulation
Python implementation for Chapter 15, Lesson 5 (System Dynamics)

Features
--------
1) Absolute-stability demonstration on the test equation y' = lambda y
2) Adaptive RK4 with step-doubling error control
3) Verification by Richardson extrapolation and observed order estimation
4) Physical sanity check on a damped oscillator (monotone energy decay)
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Callable, List, Tuple

import numpy as np


Vector = np.ndarray


def rk4_step(f: Callable[[float, Vector], Vector], t: float, y: Vector, h: float) -> Vector:
    k1 = f(t, y)
    k2 = f(t + 0.5 * h, y + 0.5 * h * k1)
    k3 = f(t + 0.5 * h, y + 0.5 * h * k2)
    k4 = f(t + h, y + h * k3)
    return y + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


@dataclass
class AdaptiveResult:
    t: np.ndarray
    y: np.ndarray
    accepted_steps: int
    rejected_steps: int


def adaptive_rk4_stepdoubling(
    f: Callable[[float, Vector], Vector],
    t0: float,
    y0: Vector,
    tf: float,
    h0: float = 0.05,
    rtol: float = 1e-6,
    atol: float = 1e-9,
    h_min: float = 1e-8,
    h_max: float = 0.5,
) -> AdaptiveResult:
    """
    Adaptive RK4 using step-doubling:
      y_big  = RK4(t, y, h)
      y_half = RK4(t+h/2, RK4(t, y, h/2), h/2)
    local error estimate: e ~= (y_half - y_big)/(2^p - 1), p=4
    """
    p = 4
    t = t0
    y = np.array(y0, dtype=float)
    h = h0

    ts: List[float] = [t]
    ys: List[Vector] = [y.copy()]
    accepted = 0
    rejected = 0

    while t < tf:
        if t + h > tf:
            h = tf - t

        y_big = rk4_step(f, t, y, h)
        y_half_1 = rk4_step(f, t, y, 0.5 * h)
        y_half_2 = rk4_step(f, t + 0.5 * h, y_half_1, 0.5 * h)

        err_est = (y_half_2 - y_big) / (2**p - 1)
        scale = atol + rtol * np.maximum(np.abs(y_half_2), np.abs(y))
        err_norm = float(np.sqrt(np.mean((err_est / scale) ** 2)))

        if err_norm <= 1.0:
            # Richardson-improved accepted value
            y = y_half_2 + err_est
            t = t + h
            ts.append(t)
            ys.append(y.copy())
            accepted += 1

            # PI-like update (simple variant)
            if err_norm == 0.0:
                factor = 2.0
            else:
                factor = 0.9 * (1.0 / err_norm) ** (1.0 / (p + 1))
            h = min(h_max, max(h_min, h * min(2.0, max(0.3, factor))))
        else:
            rejected += 1
            factor = 0.9 * (1.0 / max(err_norm, 1e-16)) ** (1.0 / (p + 1))
            h = max(h_min, h * max(0.1, min(0.5, factor)))
            if h <= h_min:
                raise RuntimeError("Step size underflow during adaptive integration.")

    return AdaptiveResult(
        t=np.array(ts),
        y=np.vstack(ys),
        accepted_steps=accepted,
        rejected_steps=rejected,
    )


def fixed_rk4(
    f: Callable[[float, Vector], Vector], t0: float, y0: Vector, tf: float, h: float
) -> Tuple[np.ndarray, np.ndarray]:
    n = int(round((tf - t0) / h))
    t = t0
    y = np.array(y0, dtype=float)
    ts = [t]
    ys = [y.copy()]
    for _ in range(n):
        y = rk4_step(f, t, y, h)
        t += h
        ts.append(t)
        ys.append(y.copy())
    return np.array(ts), np.vstack(ys)


def test_equation_stability_demo(lam: complex, h_values: List[float], n_steps: int = 50) -> None:
    """
    Demonstrate growth/decay on y' = lambda y with explicit Euler:
      y_{n+1} = (1 + h*lambda) y_n
    """
    print("\n=== Absolute stability demo (explicit Euler on y' = lambda y) ===")
    for h in h_values:
        amp = abs(1.0 + h * lam)
        y = 1.0 + 0j
        for _ in range(n_steps):
            y = y + h * lam * y
        print(f"h={h:8.4f}, |1+h*lambda|={amp:10.6f}, |y_N|={abs(y):12.6e}")


def damped_oscillator_rhs(omega_n: float, zeta: float) -> Callable[[float, Vector], Vector]:
    def f(_t: float, x: Vector) -> Vector:
        # x = [q, v]
        q, v = x
        dq = v
        dv = -2.0 * zeta * omega_n * v - (omega_n**2) * q
        return np.array([dq, dv], dtype=float)

    return f


def exact_damped_position(t: np.ndarray, omega_n: float, zeta: float, q0: float, v0: float) -> np.ndarray:
    # underdamped closed form
    if not (0.0 <= zeta < 1.0):
        raise ValueError("This exact formula is coded for 0 <= zeta < 1.")
    wd = omega_n * math.sqrt(1.0 - zeta * zeta)
    A = q0
    B = (v0 + zeta * omega_n * q0) / wd
    return np.exp(-zeta * omega_n * t) * (A * np.cos(wd * t) + B * np.sin(wd * t))


def energy(x: np.ndarray, omega_n: float) -> np.ndarray:
    q = x[:, 0]
    v = x[:, 1]
    return 0.5 * (v**2 + (omega_n**2) * q**2)


def convergence_verification() -> None:
    print("\n=== Verification via Richardson extrapolation (RK4) ===")
    omega_n = 4.0
    zeta = 0.1
    q0, v0 = 1.0, 0.0
    tf = 5.0
    f = damped_oscillator_rhs(omega_n, zeta)

    hs = [0.2, 0.1, 0.05, 0.025]
    errors = []
    for h in hs:
        t, x = fixed_rk4(f, 0.0, np.array([q0, v0]), tf, h)
        q_exact = exact_damped_position(t, omega_n, zeta, q0, v0)
        err_inf = float(np.max(np.abs(x[:, 0] - q_exact)))
        errors.append(err_inf)
        print(f"h={h:7.4f}, max|q-q_exact|={err_inf:12.6e}")

    print("\nObserved order estimates:")
    for i in range(len(errors) - 1):
        p_obs = math.log(errors[i] / errors[i + 1], 2.0)
        print(f"p_obs(h={hs[i]} -> {hs[i+1]}) = {p_obs:.4f}")

    # Richardson extrapolated final-state estimate using h and h/2
    h = 0.1
    _, xh = fixed_rk4(f, 0.0, np.array([q0, v0]), tf, h)
    _, xh2 = fixed_rk4(f, 0.0, np.array([q0, v0]), tf, h / 2.0)
    q_rich = xh2[-1, 0] + (xh2[-1, 0] - xh[-1, 0]) / (2**4 - 1)
    q_exact_tf = exact_damped_position(np.array([tf]), omega_n, zeta, q0, v0)[0]
    print(f"\nFinal-time q(T) exact          = {q_exact_tf:.10f}")
    print(f"Final-time q(T) RK4 h/2        = {xh2[-1,0]:.10f}")
    print(f"Final-time q(T) Richardson est = {q_rich:.10f}")
    print(f"Richardson abs error           = {abs(q_rich - q_exact_tf):.3e}")


def adaptive_demo() -> None:
    print("\n=== Adaptive RK4 + error control on damped oscillator ===")
    omega_n = 4.0
    zeta = 0.05
    q0, v0 = 1.0, 0.0
    tf = 12.0

    f = damped_oscillator_rhs(omega_n, zeta)
    res = adaptive_rk4_stepdoubling(
        f=f,
        t0=0.0,
        y0=np.array([q0, v0]),
        tf=tf,
        h0=0.1,
        rtol=1e-6,
        atol=1e-9,
        h_max=0.2,
    )

    E = energy(res.y, omega_n)
    energy_nonincreasing = bool(np.all(np.diff(E) <= 1e-8))

    print(f"Accepted steps: {res.accepted_steps}")
    print(f"Rejected steps: {res.rejected_steps}")
    print(f"Final state    : q={res.y[-1,0]: .6f}, v={res.y[-1,1]: .6f}")
    print(f"Energy check   : nonincreasing={energy_nonincreasing}")

    # Compare with exact displacement at the adaptive nodes
    q_exact = exact_damped_position(res.t, omega_n, zeta, q0, v0)
    err_inf = float(np.max(np.abs(res.y[:, 0] - q_exact)))
    print(f"max|q-q_exact| on adaptive mesh = {err_inf:.3e}")


def main() -> None:
    # Stability demo on a stiff-ish decay mode
    lam = -50.0 + 0.0j
    test_equation_stability_demo(lam=lam, h_values=[0.01, 0.03, 0.05, 0.06], n_steps=30)

    # Verification and adaptive control demos
    convergence_verification()
    adaptive_demo()


if __name__ == "__main__":
    main()
