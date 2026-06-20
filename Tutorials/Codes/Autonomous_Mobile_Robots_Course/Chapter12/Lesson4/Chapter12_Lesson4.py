#!/usr/bin/env python3
# Chapter12_Lesson4.py
# Graph-Based SLAM (2D Pose Graph) with Robust Kernels via IRLS (Huber)
# Minimal from-scratch educational implementation (numeric Jacobians).

import numpy as np

def wrap_angle(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def between(xi, xj):
    """Relative pose z = xi^{-1} ∘ xj expressed in i-frame.
    xi, xj: [x, y, theta]
    """
    dx = xj[0] - xi[0]
    dy = xj[1] - xi[1]
    c = np.cos(xi[2]); s = np.sin(xi[2])
    xr =  c * dx + s * dy
    yr = -s * dx + c * dy
    tr = wrap_angle(xj[2] - xi[2])
    return np.array([xr, yr, tr], dtype=float)

def huber_weight_from_s(s, delta):
    """Huber on sqrt(s): rho(s)=s if sqrt(s)<=delta else 2*delta*sqrt(s)-delta^2.
    IRLS weight w = rho'(s) = 1 if sqrt(s)<=delta else delta/sqrt(s).
    """
    r = np.sqrt(max(s, 1e-12))
    return 1.0 if r <= delta else (delta / r)

def numeric_jacobian_edge(xi, xj, z, eps=1e-6):
    """Return Jacobians of error e wrt xi and xj (3x3 each) via finite differences."""
    def err(a, b):
        zhat = between(a, b)
        e = zhat - z
        e[2] = wrap_angle(e[2])
        return e

    e0 = err(xi, xj)
    Ji = np.zeros((3, 3))
    Jj = np.zeros((3, 3))

    for k in range(3):
        d = np.zeros(3); d[k] = eps
        ei = err(xi + d, xj)
        Ji[:, k] = (ei - e0) / eps

        ej = err(xi, xj + d)
        Jj[:, k] = (ej - e0) / eps

    return e0, Ji, Jj

def solve_pose_graph_irls(
    x_init, edges,
    robust="huber", delta=1.5,
    n_iters=15, damping=1e-6,
    fix_first=True
):
    """Gauss-Newton / IRLS for 2D pose graph.
    x_init: (N,3)
    edges: list of dicts {i,j,z(3,),Omega(3x3)}
    robust: "none" or "huber"
    """
    x = x_init.copy()
    N = x.shape[0]

    # Variable ordering: optionally fix pose 0 to remove gauge freedom
    if fix_first:
        var_nodes = list(range(1, N))
        base = 1
    else:
        var_nodes = list(range(N))
        base = 0

    dim = 3 * len(var_nodes)
    node_to_col = {nid: 3 * idx for idx, nid in enumerate(var_nodes)}

    for it in range(n_iters):
        H = np.zeros((dim, dim))
        b = np.zeros((dim,))

        total_rho = 0.0
        total_s = 0.0
        for e in edges:
            i, j = e["i"], e["j"]
            z = e["z"]
            Omega = e["Omega"]

            ei, Ji, Jj = numeric_jacobian_edge(x[i], x[j], z)
            s = float(ei.T @ Omega @ ei)
            total_s += s

            if robust == "huber":
                w = huber_weight_from_s(s, delta)
                # robust cost value (for monitoring)
                r = np.sqrt(max(s, 1e-12))
                if r <= delta:
                    rho = s
                else:
                    rho = 2.0 * delta * r - delta * delta
                total_rho += rho
            else:
                w = 1.0
                total_rho += s

            W = w * Omega

            # Assemble into normal equations, skipping fixed node
            def add_block(a, Ja, bnode, Jb):
                Ha = Ja.T @ W @ Ja
                Hab = Ja.T @ W @ Jb
                ga = Ja.T @ W @ ei

                ca = node_to_col.get(a, None)
                cb = node_to_col.get(bnode, None)
                if ca is not None:
                    H[ca:ca+3, ca:ca+3] += Ha
                    b[ca:ca+3] += ga
                if ca is not None and cb is not None:
                    H[ca:ca+3, cb:cb+3] += Hab
                # NOTE: symmetric part is added when edge is processed; we'll enforce symmetry later.

            # i block
            add_block(i, Ji, j, Jj)
            # j block
            add_block(j, Jj, i, Ji)  # adds Hj and Hji, gj

        # Symmetrize, add damping (Levenberg-style)
        H = 0.5 * (H + H.T)
        H += damping * np.eye(dim)

        # Solve for dx: H dx = -b
        try:
            dx = -np.linalg.solve(H, b)
        except np.linalg.LinAlgError:
            dx = -np.linalg.lstsq(H, b, rcond=None)[0]

        # Apply update
        for nid in var_nodes:
            c = node_to_col[nid]
            x[nid, 0] += dx[c + 0]
            x[nid, 1] += dx[c + 1]
            x[nid, 2] = wrap_angle(x[nid, 2] + dx[c + 2])

        print(f"iter {it:02d}  sum_s={total_s:.3f}  sum_rho={total_rho:.3f}  |dx|={np.linalg.norm(dx):.3e}")

        if np.linalg.norm(dx) < 1e-8:
            break

    return x

def make_synthetic_graph(N=20, sigma_xy=0.05, sigma_th=0.02, seed=0):
    rng = np.random.default_rng(seed)

    # Ground truth: a smooth arc
    gt = np.zeros((N, 3))
    for k in range(1, N):
        gt[k, 0] = gt[k-1, 0] + 0.3 * np.cos(gt[k-1, 2])
        gt[k, 1] = gt[k-1, 1] + 0.3 * np.sin(gt[k-1, 2])
        gt[k, 2] = wrap_angle(gt[k-1, 2] + 0.05)

    # Odometry edges (chain)
    Omega = np.diag([1.0/(sigma_xy**2), 1.0/(sigma_xy**2), 1.0/(sigma_th**2)])
    edges = []
    for k in range(N-1):
        z = between(gt[k], gt[k+1])
        z_noisy = z + rng.normal([0,0,0], [sigma_xy, sigma_xy, sigma_th])
        z_noisy[2] = wrap_angle(z_noisy[2])
        edges.append({"i": k, "j": k+1, "z": z_noisy, "Omega": Omega})

    # One correct loop closure (optional)
    z_loop = between(gt[3], gt[N-2]) + rng.normal([0,0,0], [sigma_xy, sigma_xy, sigma_th])
    z_loop[2] = wrap_angle(z_loop[2])
    edges.append({"i": 3, "j": N-2, "z": z_loop, "Omega": Omega})

    # One outlier loop closure
    z_bad = np.array([2.0, -1.0, 1.0])  # clearly wrong relative pose
    edges.append({"i": 0, "j": N-1, "z": z_bad, "Omega": Omega})

    # Initial guess: integrate odometry only
    x0 = np.zeros_like(gt)
    for k in range(1, N):
        z = edges[k-1]["z"]
        # compose: x_k = x_{k-1} ∘ z  (approx)
        th = x0[k-1, 2]
        c = np.cos(th); s = np.sin(th)
        x0[k, 0] = x0[k-1, 0] + c*z[0] - s*z[1]
        x0[k, 1] = x0[k-1, 1] + s*z[0] + c*z[1]
        x0[k, 2] = wrap_angle(x0[k-1, 2] + z[2])

    return gt, x0, edges

if __name__ == "__main__":
    gt, x0, edges = make_synthetic_graph(N=25, seed=1)

    print("\n=== Solve without robust kernel (sensitive to outlier) ===")
    x_ls = solve_pose_graph_irls(x0, edges, robust="none", n_iters=15, damping=1e-6)

    print("\n=== Solve with Huber robust kernel (IRLS) ===")
    x_huber = solve_pose_graph_irls(x0, edges, robust="huber", delta=1.5, n_iters=15, damping=1e-6)

    # Simple endpoint comparison
    print("\nEndpoint (pose N-1):")
    print("  ground truth:", gt[-1])
    print("  least squares:", x_ls[-1])
    print("  huber:", x_huber[-1])
