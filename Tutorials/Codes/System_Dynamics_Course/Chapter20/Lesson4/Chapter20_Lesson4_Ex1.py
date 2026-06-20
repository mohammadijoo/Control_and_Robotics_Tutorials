# Chapter20_Lesson4_Ex1.py
"""
Exercise 1: Implement an embedded Runge–Kutta–Fehlberg 4(5) method (RKF45)
with adaptive step size control for a scalar test problem.

Goal:
- Understand how error estimators drive step-size selection (a core idea behind
  MATLAB ODE45 / SciPy RK45 style solvers).

We solve: y' = -y, y(0)=1 on [0, T].
Exact solution: y(t)=exp(-t).
"""

from __future__ import annotations
import math
from typing import Callable, Tuple, List


def rkf45_step(f: Callable[[float, float], float], t: float, y: float, h: float) -> Tuple[float, float]:
    """One RKF45 step. Returns (y5, err_est) where err_est = y5 - y4."""
    k1 = h * f(t, y)
    k2 = h * f(t + h/4, y + k1/4)
    k3 = h * f(t + 3*h/8, y + 3*k1/32 + 9*k2/32)
    k4 = h * f(t + 12*h/13, y + 1932*k1/2197 - 7200*k2/2197 + 7296*k3/2197)
    k5 = h * f(t + h, y + 439*k1/216 - 8*k2 + 3680*k3/513 - 845*k4/4104)
    k6 = h * f(t + h/2, y - 8*k1/27 + 2*k2 - 3544*k3/2565 + 1859*k4/4104 - 11*k5/40)

    # 4th order
    y4 = y + (25*k1/216) + (1408*k3/2565) + (2197*k4/4104) - (k5/5)

    # 5th order
    y5 = y + (16*k1/135) + (6656*k3/12825) + (28561*k4/56430) - (9*k5/50) + (2*k6/55)

    return y5, (y5 - y4)


def integrate_rkf45(
    f: Callable[[float, float], float],
    t0: float,
    y0: float,
    T: float,
    h0: float = 0.1,
    rtol: float = 1e-6,
    atol: float = 1e-9,
) -> Tuple[List[float], List[float]]:
    t = t0
    y = y0
    h = h0

    ts = [t]
    ys = [y]

    safety = 0.9
    p = 5  # higher order solution used
    h_min = 1e-10

    while t < T:
        if t + h > T:
            h = T - t

        y_new, err = rkf45_step(f, t, y, h)
        scale = atol + rtol * max(abs(y), abs(y_new))
        err_norm = abs(err) / scale

        if err_norm <= 1.0:
            # accept
            t += h
            y = y_new
            ts.append(t)
            ys.append(y)

        # adapt step
        if err_norm == 0.0:
            h *= 2.0
        else:
            h = safety * h * (1.0 / err_norm) ** (1.0 / p)

        if h < h_min:
            raise RuntimeError("Step size underflow; tolerance too strict?")

    return ts, ys


def main() -> None:
    f = lambda t, y: -y
    ts, ys = integrate_rkf45(f, 0.0, 1.0, 10.0, h0=0.2, rtol=1e-6, atol=1e-9)

    # simple report
    y_exact = math.exp(-ts[-1])
    print("t_final =", ts[-1])
    print("y_final =", ys[-1])
    print("y_exact =", y_exact)
    print("abs error =", abs(ys[-1] - y_exact))
    print("steps =", len(ts)-1)


if __name__ == "__main__":
    main()
