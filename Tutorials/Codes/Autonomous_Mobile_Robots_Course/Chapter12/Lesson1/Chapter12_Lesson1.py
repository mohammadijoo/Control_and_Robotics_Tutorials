# Chapter12_Lesson1.py
"""
Autonomous Mobile Robots (Control Engineering)
Chapter 12 — SLAM II: Graph-Based SLAM
Lesson 1 — Pose Graphs and Factor Graphs

A minimal, from-scratch 2D pose-graph optimizer (SE(2)) using Gauss–Newton.

- Variables: x_i = (x, y, theta)
- Factors/edges: relative pose measurements z_ij between nodes i and j
- Objective: sum of weighted squared residuals (Mahalanobis)

No external robotics libraries required. Uses NumPy only.
"""

from __future__ import annotations
import numpy as np

def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (a + np.pi) % (2.0 * np.pi) - np.pi
    return a

def rot(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s],
                     [s,  c]], dtype=float)

def boxplus(pose: np.ndarray, delta: np.ndarray) -> np.ndarray:
    """
    Apply a local perturbation delta = [dx, dy, dtheta] in the *local* frame.
    pose = [x, y, theta].
    """
    x, y, th = pose
    d = delta[:2]
    dth = float(delta[2])
    t_new = np.array([x, y]) + rot(th) @ d
    th_new = wrap_angle(th + dth)
    return np.array([t_new[0], t_new[1], th_new], dtype=float)

def between(x_i: np.ndarray, x_j: np.ndarray) -> np.ndarray:
    """
    Relative pose from i to j: zhat_ij = x_i^{-1} oplus x_j (SE(2) between).
    Returns [dx, dy, dtheta], where dx,dy are in frame i.
    """
    ti = x_i[:2]
    tj = x_j[:2]
    thi = float(x_i[2])
    thj = float(x_j[2])
    dt = tj - ti
    dxy = rot(thi).T @ dt
    dth = wrap_angle(thj - thi)
    return np.array([dxy[0], dxy[1], dth], dtype=float)

def edge_residual_and_jacobians(x_i: np.ndarray, x_j: np.ndarray, z_ij: np.ndarray):
    """
    Residual r_ij = z_ij - between(x_i, x_j), with angle wrapped.
    Provides analytic Jacobians wrt x_i and x_j under a small-angle local update.

    State increments are local (boxplus). The Jacobians below correspond to
    the linearization in the *global* parameterization x=[x,y,theta], but they
    are commonly used with boxplus for small deltas.
    """
    ti = x_i[:2]
    tj = x_j[:2]
    thi = float(x_i[2])
    thj = float(x_j[2])

    dt = tj - ti
    RiT = rot(thi).T

    zhat = between(x_i, x_j)
    r = z_ij - zhat
    r[2] = wrap_angle(r[2])

    # J = [[0, -1],[1, 0]] (90deg rotation)
    J = np.array([[0.0, -1.0],
                  [1.0,  0.0]], dtype=float)

    # Jacobians of residual r = z - zhat  => Jr = -Jzhat
    # zhat_xy = RiT * dt
    # d zhat_xy / d t_i = -RiT   => d r_xy / d t_i = +RiT
    # d zhat_xy / d t_j = +RiT   => d r_xy / d t_j = -RiT
    # d zhat_xy / d theta_i = d(RiT)/dtheta_i * dt = -(RiT*J)*dt
    #    => d r_xy / d theta_i = + (RiT*J*dt)
    dri_dti = RiT
    drj_dtj = -RiT
    dri_dthi = (RiT @ (J @ dt.reshape(2,1))).reshape(2,)  # 2-vector

    Ji = np.zeros((3,3), dtype=float)
    Jj = np.zeros((3,3), dtype=float)

    Ji[0:2, 0:2] = dri_dti
    Ji[0:2, 2]   = dri_dthi
    Ji[2, 2]     = 1.0  # d r_theta / d theta_i = +1 because r_theta = z_theta - (thj-thi)

    Jj[0:2, 0:2] = drj_dtj
    Jj[2, 2]     = -1.0 # d r_theta / d theta_j = -1

    return r, Ji, Jj

def build_normal_equations(poses: np.ndarray, edges: list[dict], priors: list[dict]):
    """
    Assemble H, g for Gauss–Newton: H * dx = -g.
    poses: (N,3)
    edges: list of {"i":int,"j":int,"z":(3,), "Omega":(3,3)}
    priors: list of {"i":int, "mu":(3,), "Omega":(3,3)}
    """
    N = poses.shape[0]
    dim = 3*N
    H = np.zeros((dim, dim), dtype=float)
    g = np.zeros((dim,), dtype=float)

    def sl(i):  # slice for node i in stacked vector
        return slice(3*i, 3*i+3)

    # Relative factors
    for e in edges:
        i, j = int(e["i"]), int(e["j"])
        z = np.asarray(e["z"], dtype=float).reshape(3,)
        Omega = np.asarray(e["Omega"], dtype=float).reshape(3,3)

        r, Ji, Jj = edge_residual_and_jacobians(poses[i], poses[j], z)

        # Contributions: H += J^T Ω J, g += J^T Ω r
        Hi = Ji.T @ Omega @ Ji
        Hj = Jj.T @ Omega @ Jj
        Hij = Ji.T @ Omega @ Jj
        gij = Ji.T @ Omega @ r
        gj  = Jj.T @ Omega @ r

        si, sj = sl(i), sl(j)
        H[si, si] += Hi
        H[sj, sj] += Hj
        H[si, sj] += Hij
        H[sj, si] += Hij.T
        g[si]      += gij
        g[sj]      += gj

    # Prior factors to fix gauge / anchor
    for p in priors:
        i = int(p["i"])
        mu = np.asarray(p["mu"], dtype=float).reshape(3,)
        Omega = np.asarray(p["Omega"], dtype=float).reshape(3,3)

        # residual: r = mu - x_i (wrap angle)
        r = mu - poses[i]
        r[2] = wrap_angle(r[2])
        J = np.eye(3, dtype=float) * (-1.0)  # r = mu - x  => dr/dx = -I
        si = sl(i)

        H[si, si] += J.T @ Omega @ J
        g[si]      += J.T @ Omega @ r

    return H, g

def gauss_newton_optimize(poses0: np.ndarray, edges: list[dict], priors: list[dict],
                          iters: int = 10, damping: float = 1e-8):
    poses = poses0.copy().astype(float)
    N = poses.shape[0]

    for k in range(iters):
        H, g = build_normal_equations(poses, edges, priors)
        # Levenberg-style tiny damping to stabilize
        H = H + damping * np.eye(3*N)

        dx = np.linalg.solve(H, -g)
        max_step = 0.0
        for i in range(N):
            delta = dx[3*i:3*i+3]
            poses[i] = boxplus(poses[i], delta)
            max_step = max(max_step, float(np.linalg.norm(delta)))
        print(f"iter {k:02d}: max|delta| = {max_step:.3e}")

        if max_step < 1e-9:
            break

    return poses

def demo():
    # Ground truth poses (a small loop)
    gt = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [2.0, 1.0, np.pi/2],
        [2.0, 2.0, np.pi/2],
        [1.0, 2.0, np.pi],
        [0.0, 2.0, np.pi],
        [0.0, 1.0, -np.pi/2],
    ], dtype=float)
    N = gt.shape[0]

    # Build relative measurements with noise
    rng = np.random.default_rng(4)
    sigma_xy = 0.02
    sigma_th = np.deg2rad(1.0)
    Omega = np.diag([1/sigma_xy**2, 1/sigma_xy**2, 1/sigma_th**2])

    edges = []
    for i in range(N-1):
        z = between(gt[i], gt[i+1])
        z_noisy = z + rng.normal([0,0,0], [sigma_xy, sigma_xy, sigma_th], size=3)
        z_noisy[2] = wrap_angle(z_noisy[2])
        edges.append({"i": i, "j": i+1, "z": z_noisy, "Omega": Omega})

    # Add loop closure (7 -> 0)
    z_lc = between(gt[7], gt[0])
    z_lc_noisy = z_lc + rng.normal([0,0,0], [sigma_xy, sigma_xy, sigma_th], size=3)
    z_lc_noisy[2] = wrap_angle(z_lc_noisy[2])
    edges.append({"i": 7, "j": 0, "z": z_lc_noisy, "Omega": Omega})

    # Initial guess: drifted chain integration (simulate odom drift)
    poses0 = gt.copy()
    poses0[:, 0:2] += rng.normal(0.0, 0.15, size=(N,2))
    poses0[:, 2] = np.array([wrap_angle(a) for a in (poses0[:,2] + rng.normal(0.0, np.deg2rad(5.0), size=N))])

    # Prior on node 0 to fix gauge
    prior_sigma = np.array([1e-3, 1e-3, 1e-3], dtype=float)
    Omega0 = np.diag(1/prior_sigma**2)
    priors = [{"i": 0, "mu": gt[0], "Omega": Omega0}]

    print("Optimizing...")
    est = gauss_newton_optimize(poses0, edges, priors, iters=15)

    print("\nNode  i |   gt (x,y,th)             |   est (x,y,th)")
    for i in range(N):
        print(f"{i:4d} | {gt[i]} | {est[i]}")
    return gt, poses0, est

if __name__ == "__main__":
    demo()
