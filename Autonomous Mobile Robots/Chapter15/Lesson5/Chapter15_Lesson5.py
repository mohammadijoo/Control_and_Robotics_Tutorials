# Chapter15_Lesson5.py
# Lab: Compare Local Planners in Dense Obstacles (DWA vs simplified TEB-proxy)
# Requirements: Python 3.9+, numpy, matplotlib (optional for plotting)

import math
import numpy as np

# ---------- World / geometry ----------
def corridor_y(x: float) -> float:
    return 0.9 * math.sin(0.6 * x)

def reference_path(n=120, x0=0.5, x1=11.5):
    xs = np.linspace(x0, x1, n)
    ys = np.array([corridor_y(x) for x in xs])
    return np.c_[xs, ys]

def sample_dense_world(n_obs=45, seed=0):
    rng = np.random.default_rng(seed)
    obs = []
    xmin, xmax, ymin, ymax = 0.0, 12.0, -3.0, 3.0
    tries = 0
    while len(obs) < n_obs and tries < 20000:
        tries += 1
        x = rng.uniform(xmin + 0.5, xmax - 0.5)
        y = rng.uniform(ymin + 0.5, ymax - 0.5)
        r = rng.uniform(0.12, 0.32)
        if abs(y - corridor_y(x)) < 0.55 + r:
            continue
        ok = True
        for (ox, oy, orad) in obs:
            if (x - ox) ** 2 + (y - oy) ** 2 < (r + orad + 0.05) ** 2:
                ok = False
                break
        if ok:
            obs.append((x, y, r))
    return np.array(obs, dtype=float)

def clearance(p, obs, R):
    d = np.inf
    for (ox, oy, r) in obs:
        d = min(d, math.hypot(p[0] - ox, p[1] - oy) - r - R)
    return d

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

# ---------- Unicycle ----------
def step_unicycle(x, v, w, dt):
    return np.array([x[0] + v * math.cos(x[2]) * dt,
                     x[1] + v * math.sin(x[2]) * dt,
                     wrap(x[2] + w * dt)], float)

# ---------- DWA (short) ----------
def rollout_min_clear(x, v, w, obs, R, dt, T):
    steps = max(1, int(round(T / dt)))
    xc = x.copy()
    mc = np.inf
    for _ in range(steps):
        xc = step_unicycle(xc, v, w, dt)
        mc = min(mc, clearance(xc[:2], obs, R))
        if mc <= 0:
            break
    return xc, mc

def dwa_command(x, v0, w0, path, obs, prm):
    dt, T = prm["dt"], prm["T"]
    vmin = max(0.0, v0 - prm["a"] * dt); vmax = min(prm["vmax"], v0 + prm["a"] * dt)
    wmin = max(-prm["wmax"], w0 - prm["alpha"] * dt); wmax = min(prm["wmax"], w0 + prm["alpha"] * dt)

    best = (-1e18, 0.0, 0.0)
    goal = path[-1]

    for v in np.linspace(vmin, vmax, 9):
        for w in np.linspace(wmin, wmax, 17):
            xf, mc = rollout_min_clear(x, v, w, obs, prm["R"], dt, T)
            if mc <= prm["R"] + 0.05:
                continue
            goal_dist = np.linalg.norm(xf[:2] - goal)
            J = -0.6 * goal_dist + 1.8 * mc + 0.4 * (v / (prm["vmax"] + 1e-9)) - 0.12 * (w * w)
            if J > best[0]:
                best = (J, float(v), float(w))
    return best[1], best[2]

# ---------- TEB-proxy (short band optimization) ----------
def optimize_band(x, path, obs, prm, N=12, band_len=2.6, iters=20, step=0.12):
    # sample N points ahead along x (good enough for corridor reference)
    x0 = x[0]
    xs = np.linspace(x0, x0 + band_len, N)
    pts = np.c_[xs, np.array([corridor_y(xx) for xx in xs])]
    pts[0] = x[:2]

    for _ in range(iters):
        g = np.zeros_like(pts)

        # smoothness (2nd diff)
        g[1:-1] += 0.35 * (2 * pts[1:-1] - pts[:-2] - pts[2:])

        # obstacle repulsion (nearest obstacle)
        for i in range(1, N):
            best_d = np.inf; best_u = np.zeros(2)
            for (ox, oy, r) in obs:
                dvec = pts[i] - np.array([ox, oy])
                n = np.linalg.norm(dvec) + 1e-12
                d = n - r - prm["R"]
                if d < best_d:
                    best_d = d; best_u = dvec / n
            if best_d < 0.9:
                phi = math.exp(-4.5 * (best_d - 0.9))
                g[i] += 1.8 * (-4.5 * phi) * best_u

        # time preference (shorten segments)
        g[1:] += 0.25 * (pts[1:] - pts[:-1])

        pts[1:] -= step * g[1:]
        pts[0] = x[:2]
    return pts

def teb_command(x, v0, w0, path, obs, prm):
    pts = optimize_band(x, path, obs, prm)
    d = pts[1] - pts[0]
    heading = math.atan2(d[1], d[0])
    herr = wrap(heading - x[2])
    w = max(-prm["wmax"], min(prm["wmax"], 2.2 * herr))
    v = max(0.0, min(prm["vmax"], 0.8 * (np.linalg.norm(d) / prm["dt"])))
    v *= 1.0 / (1.0 + 1.2 * abs(w))
    # accel limits
    v = max(max(0.0, v0 - prm["a"] * prm["dt"]), min(prm["vmax"], v0 + prm["a"] * prm["dt"], v))
    w = max(w0 - prm["alpha"] * prm["dt"], min(w0 + prm["alpha"] * prm["dt"], w))
    return v, w

# ---------- Benchmark ----------
def run_one(planner, seed, prm):
    obs = sample_dense_world(prm["n_obs"], seed)
    path = reference_path()
    x = np.array([0.6, 0.0, 0.0], float)
    v = 0.0; w = 0.0

    L = 0.0; cmin = np.inf; w2 = 0.0
    goal = path[-1]

    for k in range(prm["max_steps"]):
        c = clearance(x[:2], obs, prm["R"])
        cmin = min(cmin, c)
        if c <= 0:
            return dict(success=False, collision=True, time=k * prm["dt"], L=L, cmin=cmin, w2=w2)
        if np.linalg.norm(x[:2] - goal) <= prm["goal_tol"]:
            return dict(success=True, collision=False, time=k * prm["dt"], L=L, cmin=cmin, w2=w2)

        if planner == "teb":
            vcmd, wcmd = teb_command(x, v, w, path, obs, prm)
        else:
            vcmd, wcmd = dwa_command(x, v, w, path, obs, prm)

        x2 = step_unicycle(x, vcmd, wcmd, prm["dt"])
        L += float(np.linalg.norm(x2[:2] - x[:2]))
        w2 += float((wcmd ** 2) * prm["dt"])
        x, v, w = x2, vcmd, wcmd

    return dict(success=False, collision=False, time=prm["max_steps"] * prm["dt"], L=L, cmin=cmin, w2=w2)

def summarize(results):
    n = len(results)
    succ = sum(r["success"] for r in results)
    col  = sum(r["collision"] for r in results)
    def mean(key): return float(np.mean([r[key] for r in results]))
    return dict(trials=n, success_rate=succ / n, collision_rate=col / n,
                time_mean=mean("time"), L_mean=mean("L"), cmin_mean=mean("cmin"), w2_mean=mean("w2"))

def compare(trials=30, seed0=0):
    prm = dict(R=0.25, vmax=0.9, wmax=1.6, a=1.2, alpha=2.5,
               dt=0.1, T=2.0, goal_tol=0.25, max_steps=800, n_obs=45)
    for name in ["dwa", "teb"]:
        res = [run_one(name, seed0 + i, prm) for i in range(trials)]
        print(name.upper(), summarize(res))

if __name__ == "__main__":
    compare(30, 0)
