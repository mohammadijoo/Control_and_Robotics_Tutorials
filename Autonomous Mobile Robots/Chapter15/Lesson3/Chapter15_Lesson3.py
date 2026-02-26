"""
Chapter15_Lesson3.py
Dynamic Window Approach (DWA) — from-scratch reference implementation (2D unicycle).
- No ROS dependencies. Uses obstacle points and a simple receding-horizon rollout.
- You can later wrap `dwa_control()` inside a ROS2 node that publishes geometry_msgs/Twist.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple
import math
import numpy as np


@dataclass
class State:
    x: float
    y: float
    theta: float   # heading [rad]
    v: float       # linear velocity [m/s]
    w: float       # angular velocity [rad/s]


@dataclass
class Config:
    # Kinematic limits
    v_min: float = -0.2
    v_max: float =  1.0
    w_min: float = -1.5
    w_max: float =  1.5

    # Dynamic limits (accelerations)
    a_max: float = 0.8          # linear accel [m/s^2]
    a_brake: float = 1.0        # linear decel magnitude [m/s^2]
    alpha_max: float = 2.0      # angular accel [rad/s^2]

    # Discretization
    dt: float = 0.1             # control period [s]
    T: float = 2.0              # rollout horizon [s]
    v_res: float = 0.05         # sampling resolution [m/s]
    w_res: float = 0.1          # sampling resolution [rad/s]

    # Robot geometry
    robot_radius: float = 0.3   # [m]

    # Scoring weights
    w_heading: float = 0.4
    w_clear: float = 0.4
    w_speed: float = 0.2


def motion_step(s: State, v: float, w: float, dt: float) -> State:
    """Forward Euler unicycle integration."""
    nx = s.x + v * math.cos(s.theta) * dt
    ny = s.y + v * math.sin(s.theta) * dt
    ntheta = s.theta + w * dt
    return State(nx, ny, ntheta, v, w)


def dynamic_window(s: State, cfg: Config) -> Tuple[float, float, float, float]:
    """
    Dynamic window:
      - kinematic window: [v_min, v_max] x [w_min, w_max]
      - acceleration window: [v - a_brake*dt, v + a_max*dt] x [w - alpha*dt, w + alpha*dt]
      - intersection
    """
    v_low_kin, v_high_kin = cfg.v_min, cfg.v_max
    w_low_kin, w_high_kin = cfg.w_min, cfg.w_max

    v_low_dyn = s.v - cfg.a_brake * cfg.dt
    v_high_dyn = s.v + cfg.a_max * cfg.dt
    w_low_dyn = s.w - cfg.alpha_max * cfg.dt
    w_high_dyn = s.w + cfg.alpha_max * cfg.dt

    v_low = max(v_low_kin, v_low_dyn)
    v_high = min(v_high_kin, v_high_dyn)
    w_low = max(w_low_kin, w_low_dyn)
    w_high = min(w_high_kin, w_high_dyn)
    return v_low, v_high, w_low, w_high


def rollout(s0: State, v: float, w: float, cfg: Config) -> List[State]:
    n = int(cfg.T / cfg.dt)
    traj = [s0]
    s = s0
    for _ in range(n):
        s = motion_step(s, v, w, cfg.dt)
        traj.append(s)
    return traj


def min_clearance(traj: List[State], obstacles: np.ndarray) -> float:
    """
    Obstacles: array shape (M,2) of point obstacles in world coordinates.
    Returns minimum Euclidean distance from any trajectory point to any obstacle.
    """
    pts = np.array([(st.x, st.y) for st in traj], dtype=float)  # (N,2)
    # pairwise distances: (N,M)
    d = np.linalg.norm(pts[:, None, :] - obstacles[None, :, :], axis=2)
    return float(np.min(d))


def heading_score(traj: List[State], goal: Tuple[float, float]) -> float:
    """
    Heading score: cosine alignment between final heading and direction to goal.
    Range roughly [-1, 1], higher is better.
    """
    gx, gy = goal
    last = traj[-1]
    dir_to_goal = math.atan2(gy - last.y, gx - last.x)
    ang_err = wrap_to_pi(dir_to_goal - last.theta)
    return math.cos(ang_err)


def speed_score(v: float, cfg: Config) -> float:
    """Prefer higher forward speed within limits."""
    return (v - cfg.v_min) / max(1e-9, (cfg.v_max - cfg.v_min))


def admissible(v: float, clearance: float, cfg: Config) -> bool:
    """
    Stop-safe admissibility (static obstacles):
      stopping distance d_stop = v^2 / (2*a_brake)
      require clearance > d_stop + robot_radius
    """
    if clearance <= 0.0:
        return False
    d_stop = (max(0.0, v) ** 2) / max(1e-9, 2.0 * cfg.a_brake)
    return clearance > (d_stop + cfg.robot_radius)


def normalize(vals: List[float]) -> List[float]:
    lo, hi = min(vals), max(vals)
    if abs(hi - lo) < 1e-12:
        return [0.0 for _ in vals]
    return [(v - lo) / (hi - lo) for v in vals]


def dwa_control(s: State, goal: Tuple[float, float], obstacles: np.ndarray, cfg: Config) -> Tuple[float, float, List[State]]:
    """
    Compute (v*, w*) maximizing weighted score over sampled (v,w) within dynamic window.
    Returns best velocities and the associated trajectory.
    """
    vL, vU, wL, wU = dynamic_window(s, cfg)

    v_samples = np.arange(vL, vU + 1e-9, cfg.v_res)
    w_samples = np.arange(wL, wU + 1e-9, cfg.w_res)

    cand = []
    for v in v_samples:
        for w in w_samples:
            traj = rollout(s, float(v), float(w), cfg)
            clear = min_clearance(traj, obstacles) - cfg.robot_radius
            if not admissible(float(v), clear, cfg):
                continue
            h = heading_score(traj, goal)
            sp = speed_score(float(v), cfg)
            cand.append((h, clear, sp, float(v), float(w), traj))

    if not cand:
        # fallback: stop
        return 0.0, 0.0, [s]

    hs = normalize([c[0] for c in cand])
    cs = normalize([c[1] for c in cand])
    vs = normalize([c[2] for c in cand])

    best_j = -1e18
    best = cand[0]
    for i, c in enumerate(cand):
        j = cfg.w_heading * hs[i] + cfg.w_clear * cs[i] + cfg.w_speed * vs[i]
        if j > best_j:
            best_j = j
            best = c

    _, _, _, v_best, w_best, traj_best = best
    return v_best, w_best, traj_best


def wrap_to_pi(a: float) -> float:
    """Map angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def demo():
    """
    Simple closed-loop demo with point obstacles.
    Requires: numpy (and optionally matplotlib for visualization).
    """
    cfg = Config()
    s = State(x=0.0, y=0.0, theta=0.0, v=0.0, w=0.0)
    goal = (8.0, 6.0)

    # Obstacles as points
    obstacles = []
    for x in np.linspace(2.0, 7.0, 15):
        obstacles.append((x, 3.0))
    for y in np.linspace(1.0, 5.0, 12):
        obstacles.append((4.0, y))
    obs = np.array(obstacles, dtype=float)

    path = [(s.x, s.y)]
    for _ in range(600):
        v, w, _ = dwa_control(s, goal, obs, cfg)
        s = motion_step(s, v, w, cfg.dt)
        path.append((s.x, s.y))
        if math.hypot(s.x - goal[0], s.y - goal[1]) < 0.3:
            break

    print("Final state:", s)
    print("Steps:", len(path))

    try:
        import matplotlib.pyplot as plt
        path = np.array(path)
        plt.figure()
        plt.plot(path[:, 0], path[:, 1], label="trajectory")
        plt.scatter(obs[:, 0], obs[:, 1], s=15, label="obstacles")
        plt.scatter([goal[0]], [goal[1]], s=50, marker="*", label="goal")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.show()
    except Exception as e:
        print("Matplotlib plot skipped:", e)


if __name__ == "__main__":
    demo()
