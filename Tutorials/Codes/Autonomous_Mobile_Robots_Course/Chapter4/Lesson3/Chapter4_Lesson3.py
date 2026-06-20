# Chapter4_Lesson3.py
# Simple dynamic models for a ground robot: (i) velocity-lag unicycle, (ii) torque-input differential drive (optional).
# Dependencies: numpy, scipy, matplotlib (optional for plots)

from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Tuple
import numpy as np

try:
    from scipy.integrate import solve_ivp
except ImportError as e:
    raise ImportError("This script requires scipy. Install via: pip install scipy") from e


@dataclass(frozen=True)
class LagUnicycleParams:
    T_v: float = 0.25   # [s] longitudinal velocity time constant
    T_w: float = 0.20   # [s] yaw-rate time constant


def lag_unicycle_ode(t: float, x: np.ndarray,
                     v_ref: Callable[[float], float],
                     w_ref: Callable[[float], float],
                     p: LagUnicycleParams) -> np.ndarray:
    """State x = [px, py, theta, v, w]."""
    px, py, th, v, w = x
    dx = np.zeros_like(x)
    dx[0] = v * np.cos(th)
    dx[1] = v * np.sin(th)
    dx[2] = w
    dx[3] = (v_ref(t) - v) / p.T_v
    dx[4] = (w_ref(t) - w) / p.T_w
    return dx


def exact_first_order_discretization(z: float, z_ref: float, T: float, dt: float) -> float:
    """Exact discrete-time update for \dot z = (z_ref - z)/T with piecewise-constant z_ref on [k dt,(k+1) dt)."""
    a = np.exp(-dt / T)
    return a * z + (1.0 - a) * z_ref


# ---- Optional: torque-input differential-drive (very simplified) ----
@dataclass(frozen=True)
class DiffDriveTorqueParams:
    m: float = 25.0     # [kg]
    Iz: float = 1.2     # [kg m^2]
    R: float = 0.10     # [m] wheel radius
    b: float = 0.22     # [m] half wheelbase
    c_v: float = 3.5    # [N s/m] viscous longitudinal drag
    c_w: float = 0.8    # [N m s/rad] viscous yaw drag


def diffdrive_torque_ode(t: float, x: np.ndarray,
                         TR: Callable[[float], float],
                         TL: Callable[[float], float],
                         p: DiffDriveTorqueParams) -> np.ndarray:
    """State x = [px, py, theta, v, w]. Inputs are wheel torques TR, TL [N m]."""
    px, py, th, v, w = x
    # Map wheel torques to body force and yaw moment (no lateral slip; see lesson notes)
    F = (TR(t) + TL(t)) / p.R
    M = p.b * (TR(t) - TL(t)) / p.R

    dx = np.zeros_like(x)
    dx[0] = v * np.cos(th)
    dx[1] = v * np.sin(th)
    dx[2] = w
    dx[3] = (F - p.c_v * v) / p.m
    dx[4] = (M - p.c_w * w) / p.Iz
    return dx


def simulate_lag_unicycle(t0: float, tf: float, x0: np.ndarray,
                          v_ref: Callable[[float], float],
                          w_ref: Callable[[float], float],
                          p: LagUnicycleParams,
                          dt_out: float = 0.01) -> Tuple[np.ndarray, np.ndarray]:
    t_eval = np.arange(t0, tf + dt_out, dt_out)
    sol = solve_ivp(lambda t, x: lag_unicycle_ode(t, x, v_ref, w_ref, p),
                    t_span=(t0, tf), y0=x0, t_eval=t_eval, rtol=1e-8, atol=1e-10)
    if not sol.success:
        raise RuntimeError(sol.message)
    return sol.t, sol.y.T


def main() -> None:
    # Step commands: accelerate to 1.0 m/s, turn at 0.7 rad/s after 2 seconds
    def v_ref(t: float) -> float:
        return 1.0 if t >= 0.5 else 0.0

    def w_ref(t: float) -> float:
        return 0.7 if t >= 2.0 else 0.0

    p = LagUnicycleParams(T_v=0.30, T_w=0.25)
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    t, X = simulate_lag_unicycle(0.0, 8.0, x0, v_ref, w_ref, p, dt_out=0.01)

    # Print final state
    px, py, th, v, w = X[-1]
    print(f"Final pose: px={px:.3f} m, py={py:.3f} m, theta={th:.3f} rad")
    print(f"Final body velocities: v={v:.3f} m/s, w={w:.3f} rad/s")

    # Optional plot
    try:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(X[:, 0], X[:, 1])
        plt.xlabel("px [m]")
        plt.ylabel("py [m]")
        plt.title("Trajectory (lag-unicycle)")
        plt.axis("equal")

        plt.figure()
        plt.plot(t, X[:, 3], label="v")
        plt.plot(t, X[:, 4], label="w")
        plt.xlabel("t [s]")
        plt.ylabel("v, w")
        plt.legend()
        plt.title("Body velocities")
        plt.show()
    except Exception:
        pass


if __name__ == "__main__":
    main()
