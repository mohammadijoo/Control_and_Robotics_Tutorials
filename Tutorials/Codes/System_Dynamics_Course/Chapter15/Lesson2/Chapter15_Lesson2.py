# Chapter15_Lesson2.py
# Runge-Kutta Methods and Step Size Selection
# University-level demonstration: RK2, RK4, and adaptive RK4 (step-doubling)
# Optional comparison with SciPy solve_ivp (RK45) if SciPy is installed.

import math
from dataclasses import dataclass
from typing import Callable, List, Tuple

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None

try:
    from scipy.integrate import solve_ivp  # Optional library comparison
except Exception:
    solve_ivp = None


def f(t: float, y: float) -> float:
    # Test IVP: y' = -2 y + sin(t), y(0)=1
    return -2.0 * y + math.sin(t)


def y_exact(t: float) -> float:
    return (6.0 / 5.0) * math.exp(-2.0 * t) + (2.0 * math.sin(t) - math.cos(t)) / 5.0


def rk2_midpoint_step(fun: Callable[[float, float], float], t: float, y: float, h: float) -> float:
    k1 = fun(t, y)
    k2 = fun(t + 0.5 * h, y + 0.5 * h * k1)
    return y + h * k2


def rk4_step(fun: Callable[[float, float], float], t: float, y: float, h: float) -> float:
    k1 = fun(t, y)
    k2 = fun(t + 0.5 * h, y + 0.5 * h * k1)
    k3 = fun(t + 0.5 * h, y + 0.5 * h * k2)
    k4 = fun(t + h, y + h * k3)
    return y + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def integrate_fixed(
    step_fun: Callable[[Callable[[float, float], float], float, float, float], float],
    fun: Callable[[float, float], float],
    t0: float,
    tf: float,
    y0: float,
    h: float,
) -> Tuple[List[float], List[float]]:
    t = t0
    y = y0
    ts = [t]
    ys = [y]
    n_steps = int(math.ceil((tf - t0) / h))
    for _ in range(n_steps):
        h_step = min(h, tf - t)
        if h_step <= 0:
            break
        y = step_fun(fun, t, y, h_step)
        t += h_step
        ts.append(t)
        ys.append(y)
    return ts, ys


@dataclass
class AdaptiveResult:
    t: List[float]
    y: List[float]
    h_history: List[float]
    accepted: int
    rejected: int


def integrate_adaptive_rk4_stepdoubling(
    fun: Callable[[float, float], float],
    t0: float,
    tf: float,
    y0: float,
    h0: float = 0.2,
    atol: float = 1e-7,
    rtol: float = 1e-6,
    h_min: float = 1e-8,
    h_max: float = 0.5,
    safety: float = 0.9,
) -> AdaptiveResult:
    """
    Adaptive controller using RK4 with step doubling:
      y_full = RK4(h)
      y_half2 = RK4(h/2) composed twice
      error estimate (scalar) ~= |y_half2 - y_full| / (2^4 - 1) = ... / 15
    The accepted state uses Richardson-improved estimate:
      y_acc = y_half2 + (y_half2 - y_full)/15
    """
    t = t0
    y = y0
    h = h0

    ts = [t]
    ys = [y]
    hs = []
    accepted = 0
    rejected = 0

    while t < tf:
        h = min(h, tf - t)
        if h < h_min:
            raise RuntimeError("Step size fell below h_min; tolerance may be too strict.")

        y_full = rk4_step(fun, t, y, h)

        y_half = rk4_step(fun, t, y, 0.5 * h)
        y_half2 = rk4_step(fun, t + 0.5 * h, y_half, 0.5 * h)

        err_est = abs(y_half2 - y_full) / 15.0
        scale = atol + rtol * max(abs(y), abs(y_half2))
        err_norm = err_est / scale if scale > 0 else err_est

        if err_norm <= 1.0:
            # Accept
            y = y_half2 + (y_half2 - y_full) / 15.0
            t += h
            ts.append(t)
            ys.append(y)
            hs.append(h)
            accepted += 1

            # Local error estimate behaves like O(h^5) for RK4, so exponent is 1/5
            if err_norm == 0.0:
                factor = 2.0
            else:
                factor = safety * (1.0 / err_norm) ** (1.0 / 5.0)
            factor = min(2.0, max(0.2, factor))
            h = min(h_max, factor * h)
        else:
            # Reject and reduce step
            rejected += 1
            factor = safety * (1.0 / max(err_norm, 1e-16)) ** (1.0 / 5.0)
            factor = min(1.0, max(0.1, factor))
            h = max(h_min, factor * h)

    return AdaptiveResult(ts, ys, hs, accepted, rejected)


def max_error(ts: List[float], ys: List[float]) -> float:
    return max(abs(y - y_exact(t)) for t, y in zip(ts, ys))


def main() -> None:
    t0, tf, y0 = 0.0, 10.0, 1.0

    # Fixed-step runs
    h = 0.1
    t_rk2, y_rk2 = integrate_fixed(rk2_midpoint_step, f, t0, tf, y0, h)
    t_rk4, y_rk4 = integrate_fixed(rk4_step, f, t0, tf, y0, h)

    # Adaptive RK4 (step-doubling)
    adaptive = integrate_adaptive_rk4_stepdoubling(f, t0, tf, y0, h0=0.2, atol=1e-8, rtol=1e-6)

    print("=== Fixed-step results ===")
    print(f"RK2 midpoint, h={h:.3f}, steps={len(t_rk2)-1}, max error={max_error(t_rk2, y_rk2):.6e}")
    print(f"RK4,         h={h:.3f}, steps={len(t_rk4)-1}, max error={max_error(t_rk4, y_rk4):.6e}")

    print("\n=== Adaptive RK4 (step-doubling) ===")
    print(f"Accepted steps: {adaptive.accepted}")
    print(f"Rejected steps: {adaptive.rejected}")
    print(f"Max error:      {max_error(adaptive.t, adaptive.y):.6e}")
    if adaptive.h_history:
        print(f"h range:        [{min(adaptive.h_history):.3e}, {max(adaptive.h_history):.3e}]")

    if solve_ivp is not None:
        sol = solve_ivp(
            lambda t, y: [-2.0 * y[0] + math.sin(t)],
            (t0, tf),
            [y0],
            method="RK45",
            atol=1e-8,
            rtol=1e-6,
            dense_output=True,
        )
        err_scipy = 0.0
        for t in adaptive.t:
            yv = float(sol.sol(t)[0])
            err_scipy = max(err_scipy, abs(yv - y_exact(t)))
        print("\n=== SciPy solve_ivp (RK45) comparison ===")
        print(f"Status: {sol.message}")
        print(f"Internal steps: {len(sol.t)-1}")
        print(f"Max error on adaptive grid: {err_scipy:.6e}")
    else:
        print("\nSciPy not installed; skipping library comparison.")

    if plt is not None:
        t_dense = [i * (tf / 1000.0) for i in range(1001)]
        y_dense = [y_exact(t) for t in t_dense]

        plt.figure()
        plt.plot(t_dense, y_dense, label="Exact")
        plt.plot(t_rk2, y_rk2, ".", label="RK2 (h=0.1)")
        plt.plot(t_rk4, y_rk4, "-", label="RK4 (h=0.1)")
        plt.plot(adaptive.t, adaptive.y, "o", markersize=3, label="Adaptive RK4")
        plt.xlabel("t")
        plt.ylabel("y(t)")
        plt.title("Chapter15_Lesson2: RK Methods")
        plt.legend()
        plt.grid(True)

        plt.figure()
        if adaptive.h_history:
            plt.plot(adaptive.t[1:], adaptive.h_history)
            plt.xlabel("t")
            plt.ylabel("Accepted step size h")
            plt.title("Adaptive step-size history")
            plt.grid(True)
        plt.show()


if __name__ == "__main__":
    main()
