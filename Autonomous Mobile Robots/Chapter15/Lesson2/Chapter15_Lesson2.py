"""
Chapter15_Lesson2.py
Stanley Controller for Ground Vehicles (kinematic bicycle model)
- Path: polyline (x_ref, y_ref)
- Control: delta = delta_ff + e_psi + atan2(k * e_y, v + v0)
Author: Abolfazl Mohammadijoo (educational example)
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import matplotlib.pyplot as plt


def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class State:
    x: float
    y: float
    psi: float  # yaw (rad)
    v: float    # forward speed (m/s)


@dataclass
class StanleyParams:
    L: float = 2.7                 # wheelbase (m)
    k: float = 1.2                 # cross-track gain
    v0: float = 0.5                # softening speed (m/s) to avoid divide-by-zero
    max_steer: float = math.radians(30.0)  # steering saturation (rad)


def _nearest_point_on_polyline(px: float, py: float, x_ref: np.ndarray, y_ref: np.ndarray) -> Tuple[int, float, float, float]:
    """
    Return:
      idx: segment index i such that nearest point lies on segment i->i+1 (or endpoint)
      nx, ny: nearest point coordinates
      t: projection parameter in [0,1] on that segment
    """
    x1 = x_ref[:-1]
    y1 = y_ref[:-1]
    x2 = x_ref[1:]
    y2 = y_ref[1:]
    dx = x2 - x1
    dy = y2 - y1
    denom = dx*dx + dy*dy
    denom = np.where(denom < 1e-12, 1e-12, denom)

    t = ((px - x1)*dx + (py - y1)*dy) / denom
    t = np.clip(t, 0.0, 1.0)

    projx = x1 + t*dx
    projy = y1 + t*dy
    d2 = (px - projx)**2 + (py - projy)**2

    i = int(np.argmin(d2))
    return i, float(projx[i]), float(projy[i]), float(t[i])


def _segment_heading(i: int, x_ref: np.ndarray, y_ref: np.ndarray) -> float:
    dx = float(x_ref[i+1] - x_ref[i])
    dy = float(y_ref[i+1] - y_ref[i])
    return math.atan2(dy, dx)


def _curvature_at_index(i: int, x_ref: np.ndarray, y_ref: np.ndarray) -> float:
    """
    Discrete curvature estimate around point i using finite differences on indices.
    Robust enough for educational demos; for production, compute curvature vs arc-length.
    """
    n = len(x_ref)
    i0 = max(0, i-1)
    i1 = i
    i2 = min(n-1, i+1)
    x0, y0 = x_ref[i0], y_ref[i0]
    x1, y1 = x_ref[i1], y_ref[i1]
    x2, y2 = x_ref[i2], y_ref[i2]

    # Approx derivatives w.r.t index (not arc length)
    x_p = (x2 - x0) * 0.5
    y_p = (y2 - y0) * 0.5
    x_pp = x2 - 2.0*x1 + x0
    y_pp = y2 - 2.0*y1 + y0

    denom = (x_p*x_p + y_p*y_p)**1.5
    if denom < 1e-9:
        return 0.0
    kappa = (x_p*y_pp - y_p*x_pp) / denom
    return float(kappa)


def stanley_control(state: State, x_ref: np.ndarray, y_ref: np.ndarray, params: StanleyParams) -> Tuple[float, dict]:
    """
    Compute Stanley steering command.
    Convention:
      e_psi = psi_ref - psi   (desired minus actual)
      e_y   = signed cross-track error (positive if vehicle is left of path tangent)
    """
    i, nx, ny, _ = _nearest_point_on_polyline(state.x, state.y, x_ref, y_ref)
    psi_ref = _segment_heading(i, x_ref, y_ref)
    e_psi = wrap_angle(psi_ref - state.psi)

    # Signed lateral error via 2D cross product between tangent and (vehicle - nearest)
    tx = math.cos(psi_ref)
    ty = math.sin(psi_ref)
    vx = state.x - nx
    vy = state.y - ny
    cross = tx*vy - ty*vx  # >0 => vehicle left of tangent
    e_y = math.copysign(math.hypot(vx, vy), cross)

    # Optional feedforward from reference curvature
    kappa_ref = _curvature_at_index(i, x_ref, y_ref)
    delta_ff = math.atan(params.L * kappa_ref)

    # Stanley feedback term
    delta_fb = e_psi + math.atan2(params.k * e_y, state.v + params.v0)

    delta = delta_ff + delta_fb
    delta = max(-params.max_steer, min(params.max_steer, delta))

    info = {
        "i": i,
        "nx": nx,
        "ny": ny,
        "psi_ref": psi_ref,
        "e_psi": e_psi,
        "e_y": e_y,
        "kappa_ref": kappa_ref,
        "delta_ff": delta_ff,
        "delta_fb": delta_fb,
    }
    return delta, info


def step_bicycle(state: State, delta: float, dt: float, L: float) -> State:
    x = state.x + state.v * math.cos(state.psi) * dt
    y = state.y + state.v * math.sin(state.psi) * dt
    psi = wrap_angle(state.psi + state.v / L * math.tan(delta) * dt)
    return State(x=x, y=y, psi=psi, v=state.v)


def make_s_path(n: int = 400) -> Tuple[np.ndarray, np.ndarray]:
    # A simple smooth polyline path
    x = np.linspace(0.0, 50.0, n)
    y = 2.5 * np.sin(0.18 * x) + 1.0 * np.sin(0.04 * x)
    return x, y


def main() -> None:
    x_ref, y_ref = make_s_path()

    params = StanleyParams(L=2.7, k=1.4, v0=0.5, max_steer=math.radians(32))
    dt = 0.02
    T = 25.0
    steps = int(T / dt)

    state = State(x=-2.0, y=3.5, psi=math.radians(-10), v=6.0)

    xs, ys, psis, deltas = [], [], [], []
    eys, epsis = [], []

    for _ in range(steps):
        delta, info = stanley_control(state, x_ref, y_ref, params)
        state = step_bicycle(state, delta, dt, params.L)

        xs.append(state.x)
        ys.append(state.y)
        psis.append(state.psi)
        deltas.append(delta)
        eys.append(info["e_y"])
        epsis.append(info["e_psi"])

    # Plots
    plt.figure(figsize=(9, 5))
    plt.plot(x_ref, y_ref, label="reference path")
    plt.plot(xs, ys, label="vehicle trajectory")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.title("Stanley Controller: path tracking (kinematic bicycle)")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    t = np.linspace(0.0, T, steps)
    plt.figure(figsize=(9, 4))
    plt.plot(t, eys, label="e_y [m]")
    plt.plot(t, np.degrees(epsis), label="e_psi [deg]")
    plt.grid(True)
    plt.legend()
    plt.title("Tracking errors")
    plt.xlabel("time [s]")

    plt.figure(figsize=(9, 4))
    plt.plot(t, np.degrees(deltas), label="delta [deg]")
    plt.grid(True)
    plt.legend()
    plt.title("Steering command")
    plt.xlabel("time [s]")

    plt.show()


if __name__ == "__main__":
    main()
