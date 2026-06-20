# Chapter15_Lesson4.py
# Timed-Elastic-Band (TEB) local planning — educational reference implementation (SE(2) + time).
# This code is intentionally lightweight (no ROS dependency) to explain the optimization structure.

import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


def wrap_to_pi(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class CircleObstacle:
    cx: float
    cy: float
    r: float


@dataclass
class TebWeights:
    w_smooth: float = 10.0          # second-difference smoothing (positions)
    w_align: float = 2.0            # align theta with segment direction
    w_time: float = 1.0             # encourage smaller total time (sum dt)
    w_obst: float = 50.0            # obstacle clearance
    w_v: float = 10.0               # velocity bounds (soft)
    w_w: float = 5.0                # angular velocity bounds (soft)
    w_a: float = 1.0                # linear acceleration bounds (soft)
    w_alpha: float = 1.0            # angular acceleration bounds (soft)


@dataclass
class TebLimits:
    v_max: float = 0.8              # [m/s]
    w_max: float = 1.2              # [rad/s]
    a_max: float = 0.8              # [m/s^2]
    alpha_max: float = 1.5          # [rad/s^2]
    dt_min: float = 0.05            # [s]
    dt_max: float = 0.6             # [s]
    d_min: float = 0.35             # [m] (clearance)
    robot_radius: float = 0.20      # [m]


def hinge(x: float) -> float:
    """ReLU / hinge."""
    return x if x > 0.0 else 0.0


def finite_diff_jacobian(fun, x: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Numerical Jacobian of residual function fun(x) -> r."""
    r0 = fun(x)
    m = r0.size
    n = x.size
    J = np.zeros((m, n), dtype=float)
    for j in range(n):
        x1 = x.copy()
        x1[j] += eps
        r1 = fun(x1)
        J[:, j] = (r1 - r0) / eps
    return J


def gauss_newton_solve(residual_fun, x0: np.ndarray, max_iter: int = 30, lam: float = 1e-3) -> Tuple[np.ndarray, List[float]]:
    """
    Damped Gauss-Newton (LM-style) for nonlinear least squares:
        min_x 0.5 * ||r(x)||^2

    Solves: (J^T J + lam I) dx = -J^T r
    """
    x = x0.copy()
    hist = []
    for it in range(max_iter):
        r = residual_fun(x)
        cost = 0.5 * float(r @ r)
        hist.append(cost)

        J = finite_diff_jacobian(residual_fun, x)
        A = J.T @ J + lam * np.eye(J.shape[1])
        b = -(J.T @ r)

        try:
            dx = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            dx = np.linalg.lstsq(A, b, rcond=None)[0]

        # simple backtracking line search
        step = 1.0
        for _ in range(10):
            x_try = x + step * dx
            r_try = residual_fun(x_try)
            cost_try = 0.5 * float(r_try @ r_try)
            if cost_try < cost:
                x = x_try
                break
            step *= 0.5

        # small-step termination
        if np.linalg.norm(step * dx) < 1e-5:
            break

    return x, hist


def build_initial_trajectory(start: Tuple[float, float, float],
                             goal: Tuple[float, float, float],
                             N: int,
                             dt0: float = 0.2) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Straight-line initial guess:
      poses: (x_i, y_i, theta_i), i=0..N-1
      dt_i for i=0..N-2
    """
    x0, y0, th0 = start
    xg, yg, thg = goal
    xs = np.linspace(x0, xg, N)
    ys = np.linspace(y0, yg, N)
    # initialize headings from segment directions, with endpoints clamped
    thetas = np.zeros(N, dtype=float)
    thetas[0] = th0
    thetas[-1] = thg
    for i in range(1, N-1):
        dx = xs[i+1] - xs[i]
        dy = ys[i+1] - ys[i]
        thetas[i] = math.atan2(dy, dx) if (dx*dx + dy*dy) > 1e-12 else thetas[i-1]
    dts = np.full(N-1, float(dt0), dtype=float)
    return xs, ys, thetas, dts


def teb_residual_builder(start, goal, N, obstacles: List[CircleObstacle], weights: TebWeights, limits: TebLimits):
    """
    Returns residual function r(z), where z packs:
      x[1:N-1], y[1:N-1], theta[1:N-1], dt[0:N-1]
    with fixed endpoints at start/goal.
    """

    x0, y0, th0 = start
    xg, yg, thg = goal

    n_mid = N - 2
    n_dt = N - 1

    def unpack(z: np.ndarray):
        xs_mid = z[0:n_mid]
        ys_mid = z[n_mid:2*n_mid]
        th_mid = z[2*n_mid:3*n_mid]
        dts = z[3*n_mid:3*n_mid + n_dt]

        xs = np.concatenate([[x0], xs_mid, [xg]])
        ys = np.concatenate([[y0], ys_mid, [yg]])
        th = np.concatenate([[th0], th_mid, [thg]])

        return xs, ys, th, dts

    def residual(z: np.ndarray) -> np.ndarray:
        xs, ys, th, dts = unpack(z)

        res = []

        # --- dt bounds + time objective ---
        for i in range(n_dt):
            # dt bounds (soft)
            res.append(math.sqrt(weights.w_time) * dts[i])
            res.append(math.sqrt(weights.w_time) * hinge(limits.dt_min - dts[i]))
            res.append(math.sqrt(weights.w_time) * hinge(dts[i] - limits.dt_max))

        # --- smoothness: second difference in position ---
        for i in range(1, N-1):
            ddx = xs[i+1] - 2.0*xs[i] + xs[i-1]
            ddy = ys[i+1] - 2.0*ys[i] + ys[i-1]
            res.append(math.sqrt(weights.w_smooth) * ddx)
            res.append(math.sqrt(weights.w_smooth) * ddy)

        # --- align theta with segment direction (nonholonomic consistency proxy) ---
        for i in range(0, N-1):
            dx = xs[i+1] - xs[i]
            dy = ys[i+1] - ys[i]
            ang = math.atan2(dy, dx) if (dx*dx + dy*dy) > 1e-12 else th[i]
            res.append(math.sqrt(weights.w_align) * math.sin(wrap_to_pi(th[i] - ang)))

        # --- obstacle clearance ---
        for i in range(N):
            px, py = xs[i], ys[i]
            if obstacles:
                dmin = 1e9
                for obs in obstacles:
                    d = math.hypot(px - obs.cx, py - obs.cy) - (obs.r + limits.robot_radius)
                    dmin = min(dmin, d)
                res.append(math.sqrt(weights.w_obst) * hinge(limits.d_min - dmin))
            else:
                res.append(0.0)

        # --- kinematic bounds on v, w (soft) ---
        v = []
        w = []
        for i in range(N-1):
            dt = max(dts[i], 1e-6)
            dx = xs[i+1] - xs[i]
            dy = ys[i+1] - ys[i]
            ds = math.hypot(dx, dy)
            v_i = ds / dt
            w_i = wrap_to_pi(th[i+1] - th[i]) / dt
            v.append(v_i)
            w.append(w_i)

            res.append(math.sqrt(weights.w_v) * hinge(v_i - limits.v_max))
            res.append(math.sqrt(weights.w_w) * hinge(abs(w_i) - limits.w_max))

        # --- acceleration bounds (soft) ---
        for i in range(N-2):
            dtm = 0.5 * (dts[i] + dts[i+1])
            dtm = max(dtm, 1e-6)

            a_i = (v[i+1] - v[i]) / dtm
            alpha_i = (w[i+1] - w[i]) / dtm

            res.append(math.sqrt(weights.w_a) * hinge(abs(a_i) - limits.a_max))
            res.append(math.sqrt(weights.w_alpha) * hinge(abs(alpha_i) - limits.alpha_max))

        return np.asarray(res, dtype=float)

    return residual, unpack


def optimize_teb_demo():
    # Scenario (start, goal, circular obstacles)
    start = (0.0, 0.0, 0.0)
    goal = (4.0, 2.5, 0.0)
    obstacles = [
        CircleObstacle(2.0, 1.2, 0.45),
        CircleObstacle(2.8, 2.0, 0.35),
    ]

    N = 18  # number of poses
    weights = TebWeights()
    limits = TebLimits()

    xs, ys, th, dts = build_initial_trajectory(start, goal, N, dt0=0.22)

    # pack variables (exclude fixed endpoints)
    z0 = np.concatenate([
        xs[1:-1],
        ys[1:-1],
        th[1:-1],
        dts
    ])

    residual_fun, unpack = teb_residual_builder(start, goal, N, obstacles, weights, limits)

    z_opt, hist = gauss_newton_solve(residual_fun, z0, max_iter=35, lam=1e-3)
    xs_opt, ys_opt, th_opt, dts_opt = unpack(z_opt)

    return {
        "xs": xs_opt,
        "ys": ys_opt,
        "th": th_opt,
        "dts": dts_opt,
        "cost_history": hist,
        "obstacles": obstacles,
        "start": start,
        "goal": goal,
    }


if __name__ == "__main__":
    out = optimize_teb_demo()
    xs, ys = out["xs"], out["ys"]
    print("Optimized trajectory (x,y) first 5 points:")
    for i in range(min(5, len(xs))):
        print(f"{i:02d}: ({xs[i]: .3f}, {ys[i]: .3f})")
    print("Total time [s]:", float(np.sum(out["dts"])))
    print("Final cost:", out["cost_history"][-1])
