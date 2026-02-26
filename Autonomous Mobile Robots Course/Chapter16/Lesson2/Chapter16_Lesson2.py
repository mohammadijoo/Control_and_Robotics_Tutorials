"""
Chapter16_Lesson2.py
Autonomous Mobile Robots — Chapter 16, Lesson 2: Velocity Obstacles and Reciprocal Avoidance

Educational reference implementation:
- Time-horizon collision checking in relative motion
- Sampling-based velocity selection outside Velocity Obstacles (VO)
- Multi-agent simulation with synchronous (reciprocal-emergent) updates

This is NOT a full ORCA solver. For production, see RVO2 / ORCA libraries.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import random
from typing import List, Tuple

import numpy as np


@dataclass
class Agent:
    p: np.ndarray          # position (2,)
    v: np.ndarray          # velocity (2,)
    goal: np.ndarray       # goal position (2,)
    radius: float = 0.3
    v_max: float = 1.0


def norm(x: np.ndarray) -> float:
    return float(np.linalg.norm(x))


def clamp_norm(v: np.ndarray, v_max: float) -> np.ndarray:
    n = norm(v)
    if n <= v_max:
        return v
    return v * (v_max / (n + 1e-12))


def time_to_collision_in_horizon(p_rel: np.ndarray, v_rel: np.ndarray, R: float, T: float) -> Tuple[bool, float]:
    """
    Relative motion: p_rel(t) = p_rel + t v_rel.
    Collision if ||p_rel(t)|| <= R for some t in [0, T].

    Returns: (collides, t_hit)
      - collides: True if collision occurs within horizon
      - t_hit: smallest nonnegative collision time (approx; exact for quadratic), or +inf if none
    """
    # If already intersecting
    if norm(p_rel) <= R:
        return True, 0.0

    a = float(v_rel @ v_rel)
    b = 2.0 * float(p_rel @ v_rel)
    c = float(p_rel @ p_rel) - R * R

    if a < 1e-12:
        return False, float("inf")  # relative velocity ~ 0

    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return False, float("inf")

    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)

    # Earliest nonnegative intersection time
    t_hit = float("inf")
    if t1 >= 0.0:
        t_hit = t1
    elif t2 >= 0.0:
        t_hit = 0.0  # started outside, enters immediately in continuous-time view

    if 0.0 <= t_hit <= T:
        return True, t_hit
    return False, float("inf")


def sample_admissible_velocities(v_pref: np.ndarray, v_max: float, n: int = 250, spread: float = 1.2) -> List[np.ndarray]:
    """
    Samples candidate velocities in a disk of radius v_max, biased around v_pref.
    """
    cand: List[np.ndarray] = []
    # Always include v_pref clamped
    cand.append(clamp_norm(v_pref, v_max))

    for _ in range(n):
        # Gaussian around v_pref
        vx = random.gauss(float(v_pref[0]), spread * v_max / 2.0)
        vy = random.gauss(float(v_pref[1]), spread * v_max / 2.0)
        v = np.array([vx, vy], dtype=float)
        v = clamp_norm(v, v_max)
        cand.append(v)

    # Add some uniform samples
    for _ in range(max(50, n // 4)):
        ang = random.random() * 2.0 * math.pi
        r = math.sqrt(random.random()) * v_max
        cand.append(np.array([r * math.cos(ang), r * math.sin(ang)], dtype=float))

    return cand


def choose_velocity_vo(agent: Agent,
                       neighbors: List[Agent],
                       dt: float,
                       T: float,
                       w_pref: float = 1.0,
                       w_safety: float = 2.0,
                       n_samples: int = 250) -> np.ndarray:
    """
    Sampling-based VO avoidance:
    - candidate v is feasible if it does not collide with any neighbor within [0, T]
      under constant-velocity prediction.
    - objective prefers closeness to v_pref (toward goal) and larger time-to-collision.

    Reciprocal effect: all agents update synchronously each step (everyone "reacts").
    """
    # preferred velocity toward goal (simple proportional guidance)
    to_goal = agent.goal - agent.p
    dist = norm(to_goal)
    if dist < 1e-6:
        v_pref = np.zeros(2, dtype=float)
    else:
        v_pref = (to_goal / dist) * min(agent.v_max, dist / max(dt, 1e-3))

    candidates = sample_admissible_velocities(v_pref, agent.v_max, n=n_samples)

    best_v = np.zeros(2, dtype=float)
    best_cost = float("inf")

    for v in candidates:
        feasible = True
        # Safety score via minimum time-to-collision among neighbors
        min_ttc = float("inf")

        for nb in neighbors:
            p_rel = nb.p - agent.p
            v_rel = v - nb.v
            R = agent.radius + nb.radius
            coll, t_hit = time_to_collision_in_horizon(p_rel, v_rel, R, T)
            if coll:
                feasible = False
                break
            # approximate TTC (continuous-time) even if beyond T
            coll_any, t_any = time_to_collision_in_horizon(p_rel, v_rel, R, 1e6)
            if coll_any:
                min_ttc = min(min_ttc, t_any)

        if not feasible:
            continue

        # Cost: prefer v close to v_pref; also prefer larger TTC when relevant
        pref_cost = norm(v - v_pref) ** 2
        safety_cost = 0.0
        if min_ttc < float("inf"):
            safety_cost = 1.0 / (min_ttc + 1e-6)

        cost = w_pref * pref_cost + w_safety * safety_cost

        if cost < best_cost:
            best_cost = cost
            best_v = v

    # If no feasible sample, slow down (failsafe)
    if best_cost == float("inf"):
        return 0.2 * clamp_norm(agent.v, agent.v_max)

    return best_v


def simulate(n_agents: int = 8,
             steps: int = 400,
             dt: float = 0.05,
             T: float = 2.5,
             seed: int = 1) -> Tuple[np.ndarray, np.ndarray]:
    """
    Simple multi-agent crossing scenario:
    agents start on a circle, goals are opposite points.

    Returns:
      P: (steps+1, n_agents, 2) positions
      V: (steps+1, n_agents, 2) velocities
    """
    random.seed(seed)
    np.random.seed(seed)

    agents: List[Agent] = []
    R0 = 5.0
    for i in range(n_agents):
        ang = 2.0 * math.pi * i / n_agents
        p = np.array([R0 * math.cos(ang), R0 * math.sin(ang)], dtype=float)
        goal = -p
        agents.append(Agent(p=p, v=np.zeros(2, dtype=float), goal=goal, radius=0.35, v_max=1.2))

    P = np.zeros((steps + 1, n_agents, 2), dtype=float)
    V = np.zeros((steps + 1, n_agents, 2), dtype=float)

    for i, a in enumerate(agents):
        P[0, i] = a.p
        V[0, i] = a.v

    for k in range(steps):
        # Compute next velocities synchronously (reciprocal emergent)
        next_v = []
        for i, a in enumerate(agents):
            neighbors = [agents[j] for j in range(n_agents) if j != i]
            v_new = choose_velocity_vo(a, neighbors, dt=dt, T=T)
            next_v.append(v_new)

        # Update state
        for i, a in enumerate(agents):
            a.v = next_v[i]
            a.p = a.p + dt * a.v
            P[k + 1, i] = a.p
            V[k + 1, i] = a.v

    return P, V


def main() -> None:
    P, _ = simulate()
    # Save a lightweight CSV trajectory for plotting elsewhere
    # columns: step, agent, x, y
    rows = []
    for k in range(P.shape[0]):
        for i in range(P.shape[1]):
            rows.append((k, i, float(P[k, i, 0]), float(P[k, i, 1])))
    import csv
    with open("Chapter16_Lesson2_trajectories.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "agent", "x", "y"])
        w.writerows(rows)
    print("Wrote Chapter16_Lesson2_trajectories.csv")


if __name__ == "__main__":
    main()
