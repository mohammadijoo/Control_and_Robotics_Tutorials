# Chapter16_Lesson5.py
# Autonomous Mobile Robots — Chapter 16, Lesson 5
# Lab: Navigate Through Moving Crowds
#
# Minimal dependency simulator:
#   - Crowd: constant-velocity agents (optionally with small noise)
#   - Robot: unicycle model
#   - Planner: crowd-aware Dynamic Window sampling with TTC + social discomfort cost
#
# Requires: numpy, matplotlib

from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np

# -----------------------------
# Utilities
# -----------------------------
def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def rot2(phi: float) -> np.ndarray:
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s], [s, c]], dtype=float)

# -----------------------------
# Models
# -----------------------------
@dataclass
class DiscAgent:
    p: np.ndarray         # position (2,)
    v: np.ndarray         # velocity (2,)
    r: float              # radius

@dataclass
class Robot:
    p: np.ndarray         # position (2,)
    th: float             # heading
    v: float              # forward speed
    w: float              # yaw rate
    r: float              # robot radius

@dataclass
class PlannerParams:
    dt: float = 0.1
    horizon: float = 2.5
    v_max: float = 1.2
    w_max: float = 1.8
    a_v: float = 1.0      # max linear accel
    a_w: float = 2.5      # max angular accel

    # cost weights
    w_goal: float = 5.0
    w_clear: float = 2.0
    w_ttc: float = 4.0
    w_soc: float = 2.0
    w_vel: float = 0.5
    w_turn: float = 0.1
    w_smooth: float = 0.2

    # social field parameters (anisotropic around each human)
    sigma_front: float = 0.9
    sigma_side: float = 0.6
    sigma_back: float = 0.5

# -----------------------------
# Geometry: time-to-collision for discs with constant relative velocity
# -----------------------------
def ttc_discs(p_rel: np.ndarray, v_rel: np.ndarray, R: float, eps: float = 1e-9) -> float:
    """
    Earliest t >= 0 such that ||p_rel + v_rel t|| = R (disc boundaries touch).
    Returns +inf if no collision.
    """
    a = float(np.dot(v_rel, v_rel))
    b = 2.0 * float(np.dot(p_rel, v_rel))
    c = float(np.dot(p_rel, p_rel)) - R * R

    # already in collision (or touching)
    if c <= 0.0:
        return 0.0

    # no relative motion
    if a <= eps:
        return float("inf")

    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return float("inf")

    s = math.sqrt(disc)
    t1 = (-b - s) / (2.0 * a)
    t2 = (-b + s) / (2.0 * a)

    # we need the smallest nonnegative root
    if t1 >= 0.0:
        return t1
    if t2 >= 0.0:
        return t2
    return float("inf")

# -----------------------------
# Crowd scenarios
# -----------------------------
def make_crossing_crowd(n_per_stream: int = 10, speed: float = 0.9, seed: int = 0) -> list[DiscAgent]:
    """
    Two human streams crossing at (0,0):
      - Stream A: left -> right around y in [-2,2]
      - Stream B: bottom -> top around x in [-2,2]
    """
    rng = np.random.default_rng(seed)
    humans: list[DiscAgent] = []
    r = 0.25

    # left->right
    ys = rng.uniform(-2.0, 2.0, size=n_per_stream)
    xs = rng.uniform(-7.0, -3.5, size=n_per_stream)
    for x, y in zip(xs, ys):
        p = np.array([x, y], dtype=float)
        v = np.array([speed, 0.0], dtype=float)
        humans.append(DiscAgent(p=p, v=v, r=r))

    # bottom->top
    xs = rng.uniform(-2.0, 2.0, size=n_per_stream)
    ys = rng.uniform(-7.0, -3.5, size=n_per_stream)
    for x, y in zip(xs, ys):
        p = np.array([x, y], dtype=float)
        v = np.array([0.0, speed], dtype=float)
        humans.append(DiscAgent(p=p, v=v, r=r))

    return humans

def update_humans_constant_velocity(humans: list[DiscAgent], dt: float, arena: float = 9.0, noise_std: float = 0.02, seed: int | None = None) -> None:
    """
    Constant-velocity with small jitter; bounce at arena borders.
    """
    rng = np.random.default_rng(seed)
    for h in humans:
        if noise_std > 0.0:
            h.v = h.v + rng.normal(0.0, noise_std, size=2)
            sp = float(np.linalg.norm(h.v))
            if sp > 1e-6:
                h.v = h.v / sp * min(sp, 1.5)
        h.p = h.p + h.v * dt
        for k in range(2):
            if h.p[k] < -arena:
                h.p[k] = -arena
                h.v[k] = abs(h.v[k])
            if h.p[k] > arena:
                h.p[k] = arena
                h.v[k] = -abs(h.v[k])

# -----------------------------
# Planner: crowd-aware dynamic window sampling
# -----------------------------
def social_cost(robot_pos: np.ndarray, humans: list[DiscAgent], params: PlannerParams) -> float:
    """
    Sum of anisotropic Gaussians centered at each human.
    Human local frame: x forward along v, y lateral.
    """
    c = 0.0
    for h in humans:
        vnorm = float(np.linalg.norm(h.v))
        if vnorm < 1e-6:
            phi = 0.0
        else:
            phi = math.atan2(float(h.v[1]), float(h.v[0]))
        R = rot2(phi).T  # world->human frame
        rel = robot_pos - h.p
        rel_h = R @ rel
        dx, dy = float(rel_h[0]), float(rel_h[1])

        # front/back anisotropy
        if dx >= 0.0:
            sx = params.sigma_front
        else:
            sx = params.sigma_back
        sy = params.sigma_side

        q = 0.5 * ((dx / sx) ** 2 + (dy / sy) ** 2)
        c += math.exp(-q)
    return c

def simulate_unicycle(p: np.ndarray, th: float, v: float, w: float, dt: float) -> tuple[np.ndarray, float]:
    p2 = p + np.array([v * math.cos(th), v * math.sin(th)], dtype=float) * dt
    th2 = wrap_angle(th + w * dt)
    return p2, th2

def plan_control(robot: Robot, goal: np.ndarray, humans: list[DiscAgent], params: PlannerParams) -> tuple[float, float]:
    """
    Sample (v,w) in dynamic window, evaluate predicted trajectory cost.
    """
    dt = params.dt
    N = int(max(1, round(params.horizon / dt)))

    # dynamic window bounds from acceleration limits
    v_lo = clamp(robot.v - params.a_v * dt, 0.0, params.v_max)
    v_hi = clamp(robot.v + params.a_v * dt, 0.0, params.v_max)
    w_lo = clamp(robot.w - params.a_w * dt, -params.w_max, params.w_max)
    w_hi = clamp(robot.w + params.a_w * dt, -params.w_max, params.w_max)

    # preferred speed toward goal
    to_goal = goal - robot.p
    d_goal = float(np.linalg.norm(to_goal))
    v_pref = params.v_max if d_goal > 1.0 else params.v_max * d_goal

    # sampling grid
    v_samples = np.linspace(v_lo, v_hi, 9)
    w_samples = np.linspace(w_lo, w_hi, 11)

    best = (0.0, 0.0)
    best_cost = float("inf")

    for v in v_samples:
        for w in w_samples:
            p = robot.p.copy()
            th = robot.th
            ttc_min = float("inf")
            clear_min = float("inf")
            soc_sum = 0.0

            # simulate candidate
            for k in range(N):
                # predict robot next state
                p, th = simulate_unicycle(p, th, float(v), float(w), dt)
                # predict humans at that time (constant velocity)
                t = (k + 1) * dt

                # clearance & TTC using instantaneous relative velocity approximation
                for h in humans:
                    p_h = h.p + h.v * t
                    rel = p - p_h
                    dist = float(np.linalg.norm(rel)) - (robot.r + h.r)
                    clear_min = min(clear_min, dist)

                    # relative velocity: robot translational velocity in world
                    v_r = np.array([float(v) * math.cos(th), float(v) * math.sin(th)], dtype=float)
                    v_rel = v_r - h.v
                    ttc = ttc_discs(p - p_h, v_rel, robot.r + h.r)
                    ttc_min = min(ttc_min, ttc)

                soc_sum += social_cost(p, humans=[DiscAgent(p=h.p + h.v * t, v=h.v, r=h.r) for h in humans], params=params)

            # terminal goal distance
            d_term = float(np.linalg.norm(goal - p))

            # costs (avoid divisions by 0)
            eps = 1e-6
            c_goal = params.w_goal * d_term
            c_clear = params.w_clear * (1.0 / (clear_min + eps))
            c_ttc = params.w_ttc * (1.0 / (ttc_min + eps))
            c_soc = params.w_soc * (soc_sum / N)
            c_vel = params.w_vel * ((v_pref - float(v)) ** 2)
            c_turn = params.w_turn * (float(w) ** 2)
            c_smooth = params.w_smooth * ((float(v) - robot.v) ** 2 + 0.1 * (float(w) - robot.w) ** 2)

            # hard safety: reject if predicted collision in horizon
            if clear_min <= 0.0:
                continue

            cost = c_goal + c_clear + c_ttc + c_soc + c_vel + c_turn + c_smooth
            if cost < best_cost:
                best_cost = cost
                best = (float(v), float(w))

    return best

# -----------------------------
# Simulation runner
# -----------------------------
def run_episode(seed: int = 0, animate: bool = True) -> dict:
    params = PlannerParams()
    dt = params.dt

    humans = make_crossing_crowd(n_per_stream=12, speed=0.9, seed=seed)

    robot = Robot(
        p=np.array([-8.0, -8.0], dtype=float),
        th=math.radians(45.0),
        v=0.0,
        w=0.0,
        r=0.32
    )
    goal = np.array([8.0, 8.0], dtype=float)

    T_max = 70.0
    steps = int(T_max / dt)

    # logs
    traj_r = []
    traj_h = []

    for step in range(steps):
        t = step * dt

        # plan
        v_cmd, w_cmd = plan_control(robot, goal, humans, params)
        robot.v, robot.w = v_cmd, w_cmd

        # integrate robot
        robot.p, robot.th = simulate_unicycle(robot.p, robot.th, robot.v, robot.w, dt)

        # integrate humans
        update_humans_constant_velocity(humans, dt, arena=9.0, noise_std=0.01, seed=seed + step)

        traj_r.append((t, float(robot.p[0]), float(robot.p[1]), float(robot.th), float(robot.v), float(robot.w)))
        traj_h.append([(float(h.p[0]), float(h.p[1])) for h in humans])

        # termination
        if float(np.linalg.norm(goal - robot.p)) < 0.5:
            break

        # collision check
        for h in humans:
            if float(np.linalg.norm(robot.p - h.p)) <= (robot.r + h.r):
                return {"success": False, "reason": "collision", "t": t, "traj_r": traj_r, "traj_h": traj_h}

    return {"success": True, "reason": "goal", "t": traj_r[-1][0] if traj_r else 0.0, "traj_r": traj_r, "traj_h": traj_h, "goal": goal}

# -----------------------------
# Visualization
# -----------------------------
def animate_episode(result: dict) -> None:
    import matplotlib.pyplot as plt
    from matplotlib import animation

    traj_r = result["traj_r"]
    traj_h = result["traj_h"]
    goal = result.get("goal", np.array([8.0, 8.0]))

    fig, ax = plt.subplots()
    ax.set_aspect("equal", "box")
    ax.set_xlim(-9.5, 9.5)
    ax.set_ylim(-9.5, 9.5)
    ax.set_title("Crowd navigation (sampling-based crowd-aware DWA)")

    # artists
    robot_dot, = ax.plot([], [], marker="o")
    goal_dot, = ax.plot([goal[0]], [goal[1]], marker="*", markersize=12)
    humans_sc = ax.scatter([], [])
    path_line, = ax.plot([], [], linewidth=1)

    xs_r, ys_r = [], []

    def init():
        robot_dot.set_data([], [])
        humans_sc.set_offsets(np.zeros((0, 2)))
        path_line.set_data([], [])
        return robot_dot, humans_sc, path_line, goal_dot

    def update(i):
        t, x, y, th, v, w = traj_r[i]
        xs_r.append(x)
        ys_r.append(y)
        robot_dot.set_data([x], [y])
        path_line.set_data(xs_r, ys_r)

        hs = np.array(traj_h[i], dtype=float)
        humans_sc.set_offsets(hs)
        ax.set_xlabel(f"t={t:.1f}s, success={result.get('success')}, reason={result.get('reason')}")
        return robot_dot, humans_sc, path_line, goal_dot

    ani = animation.FuncAnimation(fig, update, frames=len(traj_r), init_func=init, interval=50, blit=True)
    plt.show()

if __name__ == "__main__":
    res = run_episode(seed=4, animate=False)
    print("Result:", {k: res[k] for k in ["success", "reason", "t"] if k in res})
    animate_episode(res)
