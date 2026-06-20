"""
Chapter14_Lesson4.py
System Dynamics (Control Engineering) — Chapter 14, Lesson 4
Piecewise-Linear Approximations, Saturation, Dead-Zone, and Backlash Models

This script:
1) Defines common nonlinearities used in engineering models (static + hysteretic).
2) Builds a simple closed-loop second-order plant.
3) Simulates it with an explicit RK4 integrator (kept self-contained).
4) Writes results to CSV for plotting/analysis.

Note: Numerical integration is treated formally in Chapter 15; here we use RK4 as a practical tool.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple
import math
import csv


def sat(u: float, u_max: float) -> float:
    """Hard saturation: clip to [-u_max, u_max]."""
    if u > u_max:
        return u_max
    if u < -u_max:
        return -u_max
    return u


def dead_zone(u: float, d: float) -> float:
    """
    Symmetric dead-zone of half-width d:
      y = 0                if |u| <= d
      y = u - d*sign(u)    if |u| >  d
    """
    if abs(u) <= d:
        return 0.0
    return u - d * (1.0 if u > 0.0 else -1.0)


@dataclass
class Backlash:
    """
    Rate-independent backlash (play operator) of width b.
    The output y stays within a band around the input u:
        y ∈ [u - b/2, u + b/2]
    and only changes when u pushes y to the band boundary.
    """
    b: float
    y: float = 0.0

    def step(self, u: float) -> float:
        half = 0.5 * self.b
        if u > self.y + half:
            self.y = u - half
        elif u < self.y - half:
            self.y = u + half
        return self.y


@dataclass
class PWL:
    """
    1D piecewise-linear interpolant defined by breakpoints (x_i, y_i), x_i strictly increasing.
    """
    xs: List[float]
    ys: List[float]

    def __post_init__(self) -> None:
        if len(self.xs) != len(self.ys) or len(self.xs) < 2:
            raise ValueError("PWL requires >=2 points with equal-length xs and ys.")
        for i in range(len(self.xs) - 1):
            if not (self.xs[i] < self.xs[i+1]):
                raise ValueError("xs must be strictly increasing.")

    def eval(self, x: float) -> float:
        # Extrapolate flat outside range (common in actuator maps).
        if x <= self.xs[0]:
            return self.ys[0]
        if x >= self.xs[-1]:
            return self.ys[-1]

        # Find interval (linear search; for large lists use bisect).
        for i in range(len(self.xs) - 1):
            if self.xs[i] <= x <= self.xs[i+1]:
                x0, x1 = self.xs[i], self.xs[i+1]
                y0, y1 = self.ys[i], self.ys[i+1]
                t = (x - x0) / (x1 - x0)
                return (1.0 - t) * y0 + t * y1

        # Should not reach here
        return self.ys[-1]


def rk4_step(f, t: float, x: Tuple[float, float], dt: float, u: float) -> Tuple[float, float]:
    """One step of RK4 for a 2-state system x' = f(t, x, u)."""
    x1, x2 = x

    k1 = f(t, (x1, x2), u)
    k2 = f(t + 0.5 * dt, (x1 + 0.5 * dt * k1[0], x2 + 0.5 * dt * k1[1]), u)
    k3 = f(t + 0.5 * dt, (x1 + 0.5 * dt * k2[0], x2 + 0.5 * dt * k2[1]), u)
    k4 = f(t + dt, (x1 + dt * k3[0], x2 + dt * k3[1]), u)

    xn1 = x1 + (dt / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0])
    xn2 = x2 + (dt / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
    return (xn1, xn2)


def plant_dynamics(t: float, x: Tuple[float, float], u: float,
                   wn: float = 5.0, zeta: float = 0.25, b: float = 1.0) -> Tuple[float, float]:
    """
    Second-order plant:
        x1' = x2
        x2' = -2*zeta*wn*x2 - wn^2*x1 + b*u
    """
    x1, x2 = x
    dx1 = x2
    dx2 = -2.0 * zeta * wn * x2 - (wn * wn) * x1 + b * u
    return (dx1, dx2)


def simulate(T: float = 6.0, dt: float = 1e-3) -> None:
    # Reference (step)
    def r(t: float) -> float:
        return 1.0

    # PD controller (simple)
    Kp, Kd = 18.0, 4.5

    # Nonlinearities
    u_max = 2.0     # saturation limit
    d = 0.25        # dead-zone half-width
    backlash = Backlash(b=0.20, y=0.0)

    # Optional: approximate a smooth nonlinearity g(u)=tanh(u) using PWL
    # (Illustrates PWL approximation; not required for saturation/deadzone/backlash.)
    xs = [-3.0, -1.5, -0.5, 0.0, 0.5, 1.5, 3.0]
    ys = [math.tanh(x) for x in xs]
    pwl_tanh = PWL(xs=xs, ys=ys)

    use_pwl_map = False  # if True, apply u <- pwl_tanh.eval(u) as a static map

    x = (0.0, 0.0)
    t = 0.0

    rows = []
    while t <= T + 1e-12:
        # Control law
        e = r(t) - x[0]
        u_cmd = Kp * e - Kd * x[1]

        # Apply dead-zone (e.g., valve stiction / quantization / amplifier deadband)
        u_dz = dead_zone(u_cmd, d=d)

        # Apply saturation (actuator limit)
        u_sat = sat(u_dz, u_max=u_max)

        # Optional static PWL map (e.g., linearizing an empirically measured nonlinearity)
        u_map = pwl_tanh.eval(u_sat) if use_pwl_map else u_sat

        # Apply backlash (gear train); introduces hysteresis (memory)
        u_bl = backlash.step(u_map)

        # Integrate plant
        x = rk4_step(lambda tt, xx, uu: plant_dynamics(tt, xx, uu), t, x, dt, u_bl)

        rows.append((t, x[0], x[1], u_cmd, u_dz, u_sat, u_bl))
        t += dt

    out_csv = "Chapter14_Lesson4_output.csv"
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x1", "x2", "u_cmd", "u_deadzone", "u_sat", "u_backlash"])
        w.writerows(rows)

    print(f"Wrote {out_csv} with {len(rows)} samples.")
    print("Tip: plot x1(t) and compare u_cmd vs u_backlash to see dead-zone + saturation + hysteresis effects.")


if __name__ == "__main__":
    simulate()
