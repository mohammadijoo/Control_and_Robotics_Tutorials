# Chapter16_Lesson4.py
# Prediction-Aware Local Navigation (sampling MPC with chance-constraint penalty)
# Dependencies: numpy, matplotlib (optional for plotting)

import math
import numpy as np

def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class ObstacleTrack:
    """
    Simple 2D obstacle track with constant-velocity Gaussian prediction:
      state x = [px, py, vx, vy]^T
      covariance P (4x4)
    """
    def __init__(self, mean: np.ndarray, cov: np.ndarray, radius: float, name: str = "obs"):
        self.mean = mean.astype(float).copy().reshape(4)
        self.cov = cov.astype(float).copy().reshape(4, 4)
        self.radius = float(radius)
        self.name = name

    @staticmethod
    def from_position_velocity(px, py, vx, vy, pos_sigma=0.2, vel_sigma=0.5, radius=0.35, name="obs"):
        mu = np.array([px, py, vx, vy], dtype=float)
        P = np.diag([pos_sigma**2, pos_sigma**2, vel_sigma**2, vel_sigma**2])
        return ObstacleTrack(mu, P, radius=radius, name=name)

    def predict(self, dt: float, q_pos: float = 0.05, q_vel: float = 0.2) -> None:
        """
        One-step CV prediction:
          x_{k+1} = F x_k + w,  w ~ N(0,Q)
        """
        F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=float)
        Q = np.diag([q_pos**2, q_pos**2, q_vel**2, q_vel**2]).astype(float)

        self.mean = F @ self.mean
        self.cov = F @ self.cov @ F.T + Q

def unicycle_step(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """
    x = [px, py, theta], u = [v, w]
    """
    px, py, th = float(x[0]), float(x[1]), float(x[2])
    v, w = float(u[0]), float(u[1])
    nx = px + dt * v * math.cos(th)
    ny = py + dt * v * math.sin(th)
    nth = wrap_angle(th + dt * w)
    return np.array([nx, ny, nth], dtype=float)

def chance_penalty_point(
    p_robot: np.ndarray,
    track_mu: np.ndarray,
    track_P: np.ndarray,
    R_safe: float,
    delta: float,
    eps: float = 1e-9,
) -> float:
    """
    Conservative 1D-projection chance-constraint penalty.

    Relative position r = p - o, where o ~ N(mu_o, Sigma_o)
    Treat p as deterministic, so r ~ N(mu_r, Sigma_o_pos).
    Let n = mu_r / ||mu_r||, y = n^T r ~ N(m, s^2), with:
      m = ||mu_r||, s^2 = n^T Sigma n
    Enforce m - R_safe >= z_{1-delta} s. Penalize violation squared.
    """
    # Use only 2D position block of obstacle covariance.
    Sigma = track_P[:2, :2]
    mu_o = track_mu[:2]
    mu_r = p_robot.reshape(2) - mu_o.reshape(2)
    dist = float(np.linalg.norm(mu_r)) + eps
    n = mu_r / dist

    m = dist
    s2 = float(n.T @ Sigma @ n)
    s = math.sqrt(max(s2, 0.0) + eps)

    # z_{1-delta} for common deltas (avoid scipy). Linear interp for nearby.
    # For delta in {0.1,0.05,0.02,0.01,0.005,0.001}
    z_table = {
        0.1: 1.281551565545,
        0.05: 1.644853626951,
        0.02: 2.053748910631,
        0.01: 2.326347874041,
        0.005: 2.575829303549,
        0.001: 3.090232306168,
    }
    if delta in z_table:
        z = z_table[delta]
    else:
        # crude fallback: clamp and interpolate on log10(delta)
        keys = sorted(z_table.keys())
        d = min(max(delta, keys[0]), keys[-1])
        # find bracketing
        lo = max([k for k in keys if k <= d])
        hi = min([k for k in keys if k >= d])
        if lo == hi:
            z = z_table[lo]
        else:
            t = (math.log10(d) - math.log10(lo)) / (math.log10(hi) - math.log10(lo))
            z = (1 - t) * z_table[lo] + t * z_table[hi]

    margin = z * s
    violation = max(0.0, margin - (m - R_safe))
    return violation * violation

def rollout_cost(
    x0: np.ndarray,
    u: np.ndarray,
    goal_xy: np.ndarray,
    tracks0: list,
    dt: float,
    N: int,
    weights: dict,
    delta: float,
    robot_radius: float,
) -> float:
    """
    Rollout with constant control u for N steps, while predicting obstacles forward.
    """
    x = x0.copy()
    # copy tracks so we can predict in-place
    tracks = [ObstacleTrack(t.mean.copy(), t.cov.copy(), t.radius, t.name) for t in tracks0]

    w_goal = float(weights.get("goal", 1.0))
    w_ctrl = float(weights.get("control", 0.05))
    w_risk = float(weights.get("risk", 8.0))

    cost = 0.0
    for k in range(N):
        # predict obstacles to this step (one-step ahead each loop)
        for t in tracks:
            t.predict(dt)

        x = unicycle_step(x, u, dt)
        p = x[:2]

        # quadratic goal tracking
        err = p - goal_xy.reshape(2)
        cost += w_goal * float(err.T @ err)

        # control regularization (constant control)
        cost += w_ctrl * float(u.T @ u)

        # risk penalty (sum over obstacles)
        risk_k = 0.0
        for t in tracks:
            R_safe = robot_radius + t.radius
            risk_k += chance_penalty_point(p, t.mean, t.cov, R_safe=R_safe, delta=delta)
        cost += w_risk * risk_k

    return cost

def sample_controls(v_range, w_range, n_v: int, n_w: int):
    vs = np.linspace(v_range[0], v_range[1], n_v)
    ws = np.linspace(w_range[0], w_range[1], n_w)
    for v in vs:
        for w in ws:
            yield np.array([v, w], dtype=float)

def prediction_aware_local_plan(
    x0: np.ndarray,
    goal_xy: np.ndarray,
    tracks: list,
    dt: float = 0.1,
    N: int = 20,
    v_range=(0.0, 1.0),
    w_range=(-1.5, 1.5),
    n_v: int = 11,
    n_w: int = 21,
    delta: float = 0.01,
    robot_radius: float = 0.25,
):
    """
    Returns best control u* from a sampled set, using prediction-aware risk scoring.
    """
    weights = {"goal": 1.0, "control": 0.05, "risk": 10.0}

    best_u = None
    best_J = float("inf")
    for u in sample_controls(v_range, w_range, n_v, n_w):
        J = rollout_cost(
            x0=x0,
            u=u,
            goal_xy=goal_xy,
            tracks0=tracks,
            dt=dt,
            N=N,
            weights=weights,
            delta=delta,
            robot_radius=robot_radius,
        )
        if J < best_J:
            best_J = J
            best_u = u
    return best_u, best_J

def demo():
    # Robot initial state and local goal
    x0 = np.array([0.0, 0.0, 0.0], dtype=float)
    goal = np.array([6.0, 0.0], dtype=float)

    # Two moving obstacles crossing the corridor
    tracks = [
        ObstacleTrack.from_position_velocity(px=3.0, py=1.0, vx=0.0, vy=-0.6, pos_sigma=0.15, vel_sigma=0.3, radius=0.35, name="p1"),
        ObstacleTrack.from_position_velocity(px=4.0, py=-1.2, vx=0.0, vy=0.7, pos_sigma=0.15, vel_sigma=0.3, radius=0.35, name="p2"),
    ]

    dt = 0.1
    N = 25
    u_star, J_star = prediction_aware_local_plan(
        x0=x0,
        goal_xy=goal,
        tracks=tracks,
        dt=dt,
        N=N,
        v_range=(0.0, 1.2),
        w_range=(-1.8, 1.8),
        n_v=13,
        n_w=31,
        delta=0.01,
        robot_radius=0.25,
    )
    print("Best control u* = [v, w] =", u_star, " cost =", J_star)

if __name__ == "__main__":
    demo()
