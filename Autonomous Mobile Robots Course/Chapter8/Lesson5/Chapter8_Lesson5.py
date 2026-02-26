"""Chapter8_Lesson5.py
Autonomous Mobile Robots — Chapter 8 (Particle-Filter Localization)
Lesson 5 Lab: Implement MCL on a Map

Self-contained Monte Carlo Localization (MCL) demo on a 2D occupancy grid.
Motion: sampled odometry model. Sensor: multi-beam range likelihood via ray casting.

Dependencies: numpy, matplotlib
"""

from __future__ import annotations
import math
import numpy as np
import matplotlib.pyplot as plt

def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def systematic_resample(w: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    N = w.size
    positions = (rng.random() + np.arange(N)) / N
    cdf = np.cumsum(w)
    idx = np.zeros(N, dtype=int)
    i = j = 0
    while i < N:
        if positions[i] < cdf[j]:
            idx[i] = j
            i += 1
        else:
            j += 1
    return idx

class OccupancyGrid:
    def __init__(self, grid: np.ndarray, res: float):
        self.grid = (grid > 0).astype(np.uint8)  # 1 occupied, 0 free
        self.res = float(res)
        self.H, self.W = self.grid.shape

    def is_occupied(self, gx: int, gy: int) -> bool:
        if gx < 0 or gx >= self.W or gy < 0 or gy >= self.H:
            return True
        return bool(self.grid[gy, gx])

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        return int(x / self.res), int(y / self.res)

def make_demo_map(W=220, H=160, res=0.05) -> OccupancyGrid:
    g = np.zeros((H, W), dtype=np.uint8)
    g[0, :] = 1; g[-1, :] = 1; g[:, 0] = 1; g[:, -1] = 1
    g[30:55, 40:160] = 1
    g[85:110, 20:90] = 1
    g[85:140, 140:170] = 1
    g[15:40, 180:205] = 1
    return OccupancyGrid(g, res)

def ray_cast(m: OccupancyGrid, x: float, y: float, th: float, rmax: float, step: float = 0.02) -> float:
    r = 0.0
    while r < rmax:
        wx = x + r * math.cos(th)
        wy = y + r * math.sin(th)
        gx, gy = m.world_to_grid(wx, wy)
        if m.is_occupied(gx, gy):
            return r
        r += step
    return rmax

class MCL:
    def __init__(self, m: OccupancyGrid, N=1200, rmax=6.0, seed=1):
        self.m = m
        self.N = int(N)
        self.rmax = float(rmax)
        self.rng = np.random.default_rng(seed)

        self.beams = np.deg2rad(np.linspace(-90, 90, 13))
        self.alpha1, self.alpha2, self.alpha3, self.alpha4 = 0.03, 0.01, 0.03, 0.01
        self.sigma_hit = 0.20
        self.z_hit, self.z_rand = 0.85, 0.15

        self.X = np.zeros((self.N, 3), dtype=float)     # particles
        self.w = np.full(self.N, 1.0 / self.N, dtype=float)

    def initialize_uniform(self):
        free = np.argwhere(self.m.grid == 0)
        idx = self.rng.integers(0, free.shape[0], size=self.N)
        cells = free[idx]
        self.X[:, 0] = (cells[:, 1] + 0.5) * self.m.res
        self.X[:, 1] = (cells[:, 0] + 0.5) * self.m.res
        self.X[:, 2] = self.rng.uniform(-math.pi, math.pi, size=self.N)
        self.w.fill(1.0 / self.N)

    def motion_update(self, odom: tuple[float, float, float]):
        dr1, dt, dr2 = map(float, odom)
        sr1 = math.sqrt(self.alpha1*dr1**2 + self.alpha2*dt**2)
        st  = math.sqrt(self.alpha3*dt**2  + self.alpha4*(dr1**2 + dr2**2))
        sr2 = math.sqrt(self.alpha1*dr2**2 + self.alpha2*dt**2)

        dr1h = dr1 + self.rng.normal(0.0, sr1, size=self.N)
        dth  = dt  + self.rng.normal(0.0, st,  size=self.N)
        dr2h = dr2 + self.rng.normal(0.0, sr2, size=self.N)

        x, y, th = self.X[:, 0], self.X[:, 1], self.X[:, 2]
        xn = x + dth * np.cos(th + dr1h)
        yn = y + dth * np.sin(th + dr1h)
        thn = th + dr1h + dr2h

        # reject collisions
        for i in range(self.N):
            gx, gy = self.m.world_to_grid(float(xn[i]), float(yn[i]))
            if self.m.is_occupied(gx, gy):
                xn[i], yn[i] = x[i], y[i]

        self.X[:, 0] = xn
        self.X[:, 1] = yn
        self.X[:, 2] = np.vectorize(wrap_angle)(thn)

    def sensor_update(self, z: np.ndarray):
        z = np.clip(np.asarray(z, dtype=float), 0.0, self.rmax)
        sigma = self.sigma_hit
        invs2 = 1.0/(sigma*sigma)
        norm = 1.0/(math.sqrt(2.0*math.pi)*sigma)
        unif = 1.0/self.rmax

        logw = np.zeros(self.N, dtype=float)
        for bi, ba in enumerate(self.beams):
            exp_r = np.zeros(self.N, dtype=float)
            for i in range(self.N):
                x, y, th = self.X[i]
                exp_r[i] = ray_cast(self.m, float(x), float(y), float(th + ba), self.rmax)
            dz = z[bi] - exp_r
            p = self.z_hit * norm * np.exp(-0.5*(dz*dz)*invs2) + self.z_rand * unif
            logw += np.log(p + 1e-12)

        logw -= np.max(logw)
        w = np.exp(logw)
        s = np.sum(w)
        self.w = w / s if s > 0 else np.full(self.N, 1.0/self.N)

    def neff(self) -> float:
        return 1.0 / float(np.sum(self.w * self.w))

    def resample_if_needed(self, ratio=0.55, inject=0.03):
        if self.neff() >= ratio * self.N:
            return
        idx = systematic_resample(self.w, self.rng)
        self.X = self.X[idx].copy()
        self.w.fill(1.0/self.N)

        k = int(round(inject * self.N))
        if k > 0:
            free = np.argwhere(self.m.grid == 0)
            ridx = self.rng.integers(0, free.shape[0], size=k)
            cells = free[ridx]
            self.X[:k, 0] = (cells[:, 1] + 0.5) * self.m.res
            self.X[:k, 1] = (cells[:, 0] + 0.5) * self.m.res
            self.X[:k, 2] = self.rng.uniform(-math.pi, math.pi, size=k)

    def estimate(self) -> np.ndarray:
        x = float(np.sum(self.w * self.X[:, 0]))
        y = float(np.sum(self.w * self.X[:, 1]))
        s = float(np.sum(self.w * np.sin(self.X[:, 2])))
        c = float(np.sum(self.w * np.cos(self.X[:, 2])))
        th = math.atan2(s, c)
        return np.array([x, y, th], dtype=float)

def simulate_step(p: np.ndarray, v: float, w: float, dt: float) -> np.ndarray:
    x, y, th = map(float, p)
    return np.array([x + v*math.cos(th)*dt, y + v*math.sin(th)*dt, wrap_angle(th + w*dt)], dtype=float)

def odom_from_true(p1: np.ndarray, p2: np.ndarray) -> tuple[float, float, float]:
    dx = float(p2[0] - p1[0])
    dy = float(p2[1] - p1[1])
    dtrans = math.hypot(dx, dy)
    direction = math.atan2(dy, dx)
    drot1 = wrap_angle(direction - float(p1[2])) if dtrans > 1e-9 else 0.0
    drot2 = wrap_angle(float(p2[2]) - float(p1[2]) - drot1)
    return drot1, dtrans, drot2

def range_scan(m: OccupancyGrid, p: np.ndarray, beams: np.ndarray, rmax: float, sigma: float, rng: np.random.Generator) -> np.ndarray:
    x, y, th = map(float, p)
    z = []
    for ba in beams:
        r = ray_cast(m, x, y, th + float(ba), rmax)
        r = r + float(rng.normal(0.0, sigma))
        z.append(float(np.clip(r, 0.0, rmax)))
    return np.array(z, dtype=float)

def main():
    m = make_demo_map()
    pf = MCL(m, N=1200, rmax=6.0, seed=1)
    pf.initialize_uniform()

    rng = np.random.default_rng(7)
    true = np.array([2.0, 2.0, 0.0], dtype=float)
    dt = 0.25
    T = 180
    v_cmd, w_cmd = 0.35, 0.25

    true_hist = []
    est_hist = []

    for _ in range(T):
        nxt = simulate_step(true, v_cmd, w_cmd, dt)
        gx, gy = m.world_to_grid(float(nxt[0]), float(nxt[1]))
        if m.is_occupied(gx, gy):
            nxt = simulate_step(true, 0.0, 0.8, dt)

        odom = odom_from_true(true, nxt)
        z = range_scan(m, nxt, pf.beams, pf.rmax, sigma=0.05, rng=rng)

        pf.motion_update(odom)
        pf.sensor_update(z)
        pf.resample_if_needed(ratio=0.55, inject=0.03)

        true = nxt
        true_hist.append(true.copy())
        est_hist.append(pf.estimate())

    true_hist = np.array(true_hist)
    est_hist = np.array(est_hist)

    fig, ax = plt.subplots(figsize=(9, 6))
    ax.imshow(m.grid, origin="lower", cmap="gray_r",
              extent=[0, m.W*m.res, 0, m.H*m.res])
    ax.plot(true_hist[:,0], true_hist[:,1], label="true")
    ax.plot(est_hist[:,0], est_hist[:,1], label="MCL estimate")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("MCL on an Occupancy Grid Map")
    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
