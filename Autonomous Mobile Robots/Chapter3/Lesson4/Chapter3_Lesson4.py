#!/usr/bin/env python3
"""
Chapter 3 — Lesson 4: Motion Primitives for Ground Vehicles (conceptual use)

This script builds a small library of constant-curvature motion primitives for a
car-like (kinematic bicycle) model and demonstrates:
  1) analytic endpoint computation,
  2) sampling a trajectory for visualization,
  3) snapping endpoints to a simple SE(2) lattice.

Dependencies:
  - numpy
  - matplotlib (optional, for plotting)
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple, Dict
import json
import math
import numpy as np

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None


@dataclass(frozen=True)
class State:
    x: float
    y: float
    theta: float  # heading [rad]


@dataclass(frozen=True)
class Primitive:
    name: str
    v: float
    kappa: float
    T: float
    cost: float
    dx: float
    dy: float
    dtheta: float


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def endpoint_constant_curvature(v: float, kappa: float, T: float) -> Tuple[float, float, float]:
    dtheta = v * kappa * T
    if abs(kappa) < 1e-12:
        return v * T, 0.0, dtheta
    return math.sin(dtheta) / kappa, (1.0 - math.cos(dtheta)) / kappa, dtheta


def sample_trajectory(state0: State, prim: Primitive, dt: float = 0.02) -> List[State]:
    x, y, th = state0.x, state0.y, state0.theta
    t = 0.0
    out = [state0]
    while t < prim.T - 1e-12:
        h = min(dt, prim.T - t)
        x += prim.v * math.cos(th) * h
        y += prim.v * math.sin(th) * h
        th = wrap_to_pi(th + prim.v * prim.kappa * h)
        out.append(State(x, y, th))
        t += h
    return out


def apply_primitive(state: State, prim: Primitive) -> State:
    c, s = math.cos(state.theta), math.sin(state.theta)
    x = state.x + c * prim.dx - s * prim.dy
    y = state.y + s * prim.dx + c * prim.dy
    th = wrap_to_pi(state.theta + prim.dtheta)
    return State(x, y, th)


def snap_to_lattice(state: State, xy_res: float, n_theta: int) -> State:
    xq = round(state.x / xy_res) * xy_res
    yq = round(state.y / xy_res) * xy_res
    dth = 2 * math.pi / n_theta
    thq = round(state.theta / dth) * dth
    thq = wrap_to_pi(thq)
    return State(xq, yq, thq)


def build_primitives(L: float, delta_set: List[float], v: float, arc_length: float,
                     w_kappa: float = 0.2, w_time: float = 0.1) -> List[Primitive]:
    T = arc_length / max(v, 1e-9)
    prims: List[Primitive] = []
    for delta in delta_set:
        kappa = math.tan(delta) / L
        dx, dy, dtheta = endpoint_constant_curvature(v, kappa, T)
        cost = arc_length + w_kappa * abs(kappa) * arc_length + w_time * T
        prims.append(Primitive(
            name=f"delta={delta:+.3f}_kappa={kappa:+.3f}",
            v=v, kappa=kappa, T=T, cost=cost, dx=dx, dy=dy, dtheta=dtheta
        ))
    return prims


def main() -> None:
    L = 0.35
    R_min = 1.2
    kappa_max = 1.0 / R_min
    delta_max = math.atan(kappa_max * L)

    v = 0.6
    arc_length = 0.5
    delta_set = np.linspace(-delta_max, delta_max, 5).tolist()

    prims = build_primitives(L, delta_set, v, arc_length)

    print("Primitive library:")
    for p in prims:
        print(f"  {p.name:>20s} : dx={p.dx:+.3f}, dy={p.dy:+.3f}, dtheta={p.dtheta:+.3f}, cost={p.cost:.3f}")

    s0 = State(1.0, 2.0, math.radians(30))
    print("\nExpansion from state:", s0)
    for p in prims:
        s1 = apply_primitive(s0, p)
        s1q = snap_to_lattice(s1, xy_res=0.25, n_theta=16)
        print(f"  via {p.name:>20s} -> {s1} -> snapped {s1q}")

    with open("Chapter3_Lesson4_primitives.json", "w", encoding="utf-8") as f:
        json.dump([p.__dict__ for p in prims], f, indent=2)

    if plt is not None:
        plt.figure()
        for p in prims:
            traj = sample_trajectory(State(0.0, 0.0, 0.0), p, dt=0.01)
            plt.plot([s.x for s in traj], [s.y for s in traj], label=p.name)
        plt.axis("equal"); plt.grid(True)
        plt.title("Chapter 3 Lesson 4: Motion primitives (local frame)")
        plt.legend(fontsize=7)
        plt.show()


if __name__ == "__main__":
    main()
