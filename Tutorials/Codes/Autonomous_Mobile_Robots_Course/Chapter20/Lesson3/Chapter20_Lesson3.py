# Chapter20_Lesson3.py
"""
Localization + Mapping Integration (Capstone AMR)
------------------------------------------------
Educational implementation of a loosely coupled pipeline:
  1) Differential-drive prediction (wheel + yaw-rate)
  2) Landmark EKF correction
  3) Occupancy-grid log-odds mapping from 2D range scans
The code is intentionally compact and readable for course use.
"""

from dataclasses import dataclass
import math
import numpy as np


@dataclass
class Config:
    dt: float = 0.1
    wheel_base: float = 0.34
    q_v: float = 0.02          # process noise on linear velocity
    q_w: float = 0.03          # process noise on yaw rate
    r_range: float = 0.15      # landmark range noise
    r_bearing: float = 0.03    # landmark bearing noise
    grid_res: float = 0.20
    grid_size: int = 120       # square grid
    l_occ: float = 0.85
    l_free: float = -0.40
    l_min: float = -4.0
    l_max: float = 4.0


def wrap_angle(a: float) -> float:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class EKFLocalizer:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.x = np.array([1.0, 1.0, 0.0], dtype=float)  # [x, y, theta]
        self.P = np.diag([0.05, 0.05, 0.02])

    def predict(self, v: float, w: float):
        dt = self.cfg.dt
        x, y, th = self.x

        # State propagation
        if abs(w) < 1e-6:
            nx = x + v * math.cos(th) * dt
            ny = y + v * math.sin(th) * dt
            nth = th
        else:
            nx = x + (v / w) * (math.sin(th + w * dt) - math.sin(th))
            ny = y - (v / w) * (math.cos(th + w * dt) - math.cos(th))
            nth = th + w * dt

        self.x = np.array([nx, ny, wrap_angle(nth)])

        # Jacobian wrt state (first-order)
        F = np.eye(3)
        F[0, 2] = -v * math.sin(th) * dt
        F[1, 2] =  v * math.cos(th) * dt

        # Jacobian wrt control noise [delta_v, delta_w]
        G = np.zeros((3, 2))
        G[0, 0] = math.cos(th) * dt
        G[1, 0] = math.sin(th) * dt
        G[2, 1] = dt
        Q = np.diag([self.cfg.q_v**2, self.cfg.q_w**2])

        self.P = F @ self.P @ F.T + G @ Q @ G.T

    def correct_landmark(self, z_range: float, z_bearing: float, landmark_xy):
        lx, ly = landmark_xy
        x, y, th = self.x
        dx = lx - x
        dy = ly - y
        q = dx * dx + dy * dy
        if q < 1e-9:
            return

        zhat = np.array([math.sqrt(q), wrap_angle(math.atan2(dy, dx) - th)])

        H = np.array([
            [-dx / math.sqrt(q), -dy / math.sqrt(q), 0.0],
            [dy / q,            -dx / q,           -1.0],
        ])

        R = np.diag([self.cfg.r_range**2, self.cfg.r_bearing**2])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        innov = np.array([z_range - zhat[0], wrap_angle(z_bearing - zhat[1])])

        # Chi-square gating for 2D measurement
        d2 = float(innov.T @ np.linalg.inv(S) @ innov)
        if d2 > 9.21:  # approx chi-square(2, 0.99)
            return

        self.x = self.x + K @ innov
        self.x[2] = wrap_angle(self.x[2])

        I = np.eye(3)
        # Joseph form for numerical stability and PSD covariance
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T


class OccupancyGrid:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.logodds = np.zeros((cfg.grid_size, cfg.grid_size), dtype=float)
        self.origin_world = np.array([0.0, 0.0])  # lower-left corner in meters

    def world_to_grid(self, x: float, y: float):
        gx = int((x - self.origin_world[0]) / self.cfg.grid_res)
        gy = int((y - self.origin_world[1]) / self.cfg.grid_res)
        return gx, gy

    def in_bounds(self, gx: int, gy: int) -> bool:
        n = self.cfg.grid_size
        return 0 <= gx < n and 0 <= gy < n

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return points

    def update_scan(self, pose, scan_ranges, scan_angles, max_range=8.0):
        rx, ry, rth = pose
        g0x, g0y = self.world_to_grid(rx, ry)
        if not self.in_bounds(g0x, g0y):
            return

        for r, a in zip(scan_ranges, scan_angles):
            if r <= 0.05:
                continue
            rr = min(r, max_range)
            ex = rx + rr * math.cos(rth + a)
            ey = ry + rr * math.sin(rth + a)
            g1x, g1y = self.world_to_grid(ex, ey)
            if not self.in_bounds(g1x, g1y):
                continue

            ray = self.bresenham(g0x, g0y, g1x, g1y)
            # Free cells along the ray (except terminal)
            for (cx, cy) in ray[:-1]:
                self.logodds[cy, cx] = np.clip(
                    self.logodds[cy, cx] + self.cfg.l_free,
                    self.cfg.l_min,
                    self.cfg.l_max,
                )
            # Occupied terminal cell only if not max-range truncation
            if r < max_range:
                cx, cy = ray[-1]
                self.logodds[cy, cx] = np.clip(
                    self.logodds[cy, cx] + self.cfg.l_occ,
                    self.cfg.l_min,
                    self.cfg.l_max,
                )

    def occupancy_probability(self):
        return 1.0 / (1.0 + np.exp(-self.logodds))


def simulate_controls(k: int):
    # Smooth commands for a capstone-style indoor path
    v = 0.35 + 0.05 * math.sin(0.05 * k)
    w = 0.25 * math.sin(0.03 * k)
    return v, w


def synthetic_landmark_measurement(true_pose, landmark):
    tx, ty, tth = true_pose
    lx, ly = landmark
    dx, dy = lx - tx, ly - ty
    rng = math.hypot(dx, dy) + np.random.normal(0.0, 0.08)
    brg = wrap_angle(math.atan2(dy, dx) - tth + np.random.normal(0.0, 0.02))
    return rng, brg


def synthetic_scan(true_pose, n_beams=61, max_range=8.0):
    # Simple simulated room with one internal obstacle rectangle
    x, y, th = true_pose
    angles = np.linspace(-1.2, 1.2, n_beams)
    ranges = np.full_like(angles, max_range, dtype=float)

    # World boundaries and one box obstacle (axis-aligned)
    segments = [
        ((0.0, 0.0), (12.0, 0.0)), ((12.0, 0.0), (12.0, 12.0)),
        ((12.0, 12.0), (0.0, 12.0)), ((0.0, 12.0), (0.0, 0.0)),
        ((4.5, 4.0), (7.5, 4.0)), ((7.5, 4.0), (7.5, 6.5)),
        ((7.5, 6.5), (4.5, 6.5)), ((4.5, 6.5), (4.5, 4.0)),
    ]

    def ray_segment_intersection(px, py, dx, dy, ax, ay, bx, by):
        sx, sy = (bx - ax), (by - ay)
        denom = (-sx * dy + dx * sy)
        if abs(denom) < 1e-9:
            return None
        s = (-dy * (px - ax) + dx * (py - ay)) / denom
        t = ( sx * (py - ay) - sy * (px - ax)) / denom
        if 0.0 <= s <= 1.0 and t >= 0.0:
            return t
        return None

    for i, a in enumerate(angles):
        ang = th + a
        dx, dy = math.cos(ang), math.sin(ang)
        best = max_range
        for (p0, p1) in segments:
            t = ray_segment_intersection(x, y, dx, dy, p0[0], p0[1], p1[0], p1[1])
            if t is not None and t < best:
                best = t
        ranges[i] = max(0.05, best + np.random.normal(0.0, 0.03))
    return ranges, angles


def main():
    np.random.seed(4)
    cfg = Config()
    ekf = EKFLocalizer(cfg)
    grid = OccupancyGrid(cfg)

    # Ground-truth pose and landmarks
    x_true = np.array([1.0, 1.0, 0.0], dtype=float)
    landmarks = [(2.0, 10.0), (10.0, 2.5), (9.5, 10.5), (2.0, 5.5)]

    traj_true = []
    traj_est = []

    for k in range(240):
        v_cmd, w_cmd = simulate_controls(k)

        # Truth propagation with noise (plant)
        x_true[0] += v_cmd * math.cos(x_true[2]) * cfg.dt
        x_true[1] += v_cmd * math.sin(x_true[2]) * cfg.dt
        x_true[2] = wrap_angle(x_true[2] + w_cmd * cfg.dt + np.random.normal(0.0, 0.005))

        # Localization prediction with noisy control
        v_meas = v_cmd + np.random.normal(0.0, 0.02)
        w_meas = w_cmd + np.random.normal(0.0, 0.01)
        ekf.predict(v_meas, w_meas)

        # Landmark correction every 5 steps (simulate detector)
        if k % 5 == 0:
            for lm in landmarks:
                rng, brg = synthetic_landmark_measurement(x_true, lm)
                if rng < 7.5:
                    ekf.correct_landmark(rng, brg, lm)

        # Mapping update every step using estimated pose (typical online pipeline)
        scan_r, scan_a = synthetic_scan(x_true)
        grid.update_scan(ekf.x.copy(), scan_r, scan_a)

        traj_true.append(x_true.copy())
        traj_est.append(ekf.x.copy())

    traj_true = np.array(traj_true)
    traj_est = np.array(traj_est)

    pos_rmse = np.sqrt(np.mean(np.sum((traj_true[:, :2] - traj_est[:, :2]) ** 2, axis=1)))
    yaw_rmse = np.sqrt(np.mean((np.unwrap(traj_true[:, 2]) - np.unwrap(traj_est[:, 2])) ** 2))

    print("Final estimated pose:", ekf.x)
    print("Final covariance diag:", np.diag(ekf.P))
    print("Position RMSE [m]:", round(float(pos_rmse), 4))
    print("Yaw RMSE [rad]:", round(float(yaw_rmse), 4))

    occ_prob = grid.occupancy_probability()
    print("Map occupancy probability stats:",
          "min=", round(float(np.min(occ_prob)), 3),
          "max=", round(float(np.max(occ_prob)), 3),
          "mean=", round(float(np.mean(occ_prob)), 3))


if __name__ == "__main__":
    main()
