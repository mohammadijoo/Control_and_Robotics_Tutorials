# Chapter12_Lesson5.py
# Lab: Build and Optimize a 2D Pose Graph (SE(2)) using Gauss-Newton / Levenberg-Marquardt
# Dependencies: numpy, scipy
#
# Run:
#   python Chapter12_Lesson5.py
#
# This implementation is from-scratch (no g2o/GTSAM), intended for educational use.

from __future__ import annotations
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
from scipy import sparse
from scipy.sparse.linalg import spsolve


def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (a + math.pi) % (2 * math.pi) - math.pi
    # make pi inclusive on the right if you prefer; here keep (-pi, pi]
    if a <= -math.pi:
        a += 2 * math.pi
    return a


def rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


SKEW = np.array([[0.0, -1.0], [1.0, 0.0]], dtype=float)


@dataclass
class EdgeSE2:
    i: int
    j: int
    z: np.ndarray   # measurement [dx, dy, dtheta] in i frame
    Omega: np.ndarray  # information (3x3), symmetric PD


def se2_compose(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Compose SE2 in minimal coords: x ⊕ y (apply x then y)."""
    tx, ty, th = float(x[0]), float(x[1]), float(x[2])
    ux, uy, uh = float(y[0]), float(y[1]), float(y[2])
    R = rot2(th)
    t = np.array([tx, ty]) + R @ np.array([ux, uy])
    return np.array([t[0], t[1], wrap_angle(th + uh)], dtype=float)


def se2_inverse(x: np.ndarray) -> np.ndarray:
    tx, ty, th = float(x[0]), float(x[1]), float(x[2])
    R = rot2(th)
    t = np.array([tx, ty])
    tinv = -(R.T @ t)
    return np.array([tinv[0], tinv[1], wrap_angle(-th)], dtype=float)


def se2_between(xi: np.ndarray, xj: np.ndarray) -> np.ndarray:
    """Relative pose from i to j: xi^{-1} ⊕ xj."""
    return se2_compose(se2_inverse(xi), xj)


def pose_graph_error(x: np.ndarray, edges: List[EdgeSE2], N: int) -> float:
    """Total chi^2 cost: sum e^T Omega e."""
    cost = 0.0
    for e in edges:
        xi = x[3*e.i:3*e.i+3]
        xj = x[3*e.j:3*e.j+3]
        zhat = se2_between(xi, xj)               # predicted relative
        # error = z^{-1} ⊕ zhat
        zinv = se2_inverse(e.z)
        err = se2_compose(zinv, zhat)
        err[2] = wrap_angle(err[2])
        cost += float(err.T @ e.Omega @ err)
    return cost


def linearize_edge(xi: np.ndarray, xj: np.ndarray, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute residual e and Jacobians A, B for an SE(2) relative constraint.

    We use the common "measurement-in-i-frame" model:
      zhat_t = R_i^T (t_j - t_i)
      zhat_th = th_j - th_i
    and define the error as:
      e_t = R_z^T ( zhat_t - z_t )
      e_th = wrap( zhat_th - z_th )

    This corresponds to e = Log( z^{-1} * (x_i^{-1} x_j) ) under small-angle approximation.

    Returns:
      e (3,), A (3x3) wrt xi, B (3x3) wrt xj.
    """
    ti = xi[0:2]; tj = xj[0:2]
    thi = float(xi[2]); thj = float(xj[2])

    Ri = rot2(thi)
    Rz = rot2(float(z[2]))

    dt = tj - ti
    zhat_t = Ri.T @ dt
    zhat_th = wrap_angle(thj - thi)

    e_t = Rz.T @ (zhat_t - z[0:2])
    e_th = wrap_angle(zhat_th - float(z[2]))

    evec = np.array([e_t[0], e_t[1], e_th], dtype=float)

    # Jacobians
    A = np.zeros((3, 3), dtype=float)
    B = np.zeros((3, 3), dtype=float)

    # translation wrt positions
    A[0:2, 0:2] = -Rz.T @ Ri.T
    B[0:2, 0:2] =  Rz.T @ Ri.T

    # translation wrt theta_i
    # d(Ri^T dt)/dtheta = -Ri^T S dt
    d_ri = -Ri.T @ (SKEW @ dt)
    A[0:2, 2] = (Rz.T @ d_ri)

    # rotation residual
    A[2, 2] = -1.0
    B[2, 2] =  1.0

    return evec, A, B


def build_normal_equations(x: np.ndarray, edges: List[EdgeSE2], N: int, lambda_damp: float = 0.0):
    """
    Assemble sparse normal equations:
      H dx = -b
    with H = sum J^T Omega J, b = sum J^T Omega e.
    """
    dim = 3 * N
    H = sparse.lil_matrix((dim, dim), dtype=float)
    b = np.zeros(dim, dtype=float)

    for edge in edges:
        i, j = edge.i, edge.j
        xi = x[3*i:3*i+3]
        xj = x[3*j:3*j+3]

        e, A, B = linearize_edge(xi, xj, edge.z)
        Om = edge.Omega

        ii = slice(3*i, 3*i+3)
        jj = slice(3*j, 3*j+3)

        H[ii, ii] += A.T @ Om @ A
        H[ii, jj] += A.T @ Om @ B
        H[jj, ii] += B.T @ Om @ A
        H[jj, jj] += B.T @ Om @ B

        b[ii] += A.T @ Om @ e
        b[jj] += B.T @ Om @ e

    if lambda_damp > 0.0:
        # Levenberg-Marquardt: H + lambda * diag(H)
        diag = H.diagonal()
        H.setdiag(diag + lambda_damp * (diag + 1e-12))

    # Gauge fix: anchor node 0 (set dx0 = 0) by setting corresponding rows/cols to identity
    for k in range(3):
        idx = k  # node 0
        H[idx, :] = 0.0
        H[:, idx] = 0.0
        H[idx, idx] = 1.0
        b[idx] = 0.0

    return H.tocsr(), b


def solve_pose_graph(
    x0: np.ndarray,
    edges: List[EdgeSE2],
    N: int,
    max_iters: int = 20,
    tol: float = 1e-6,
    lm: bool = True,
) -> np.ndarray:
    x = x0.copy()
    lam = 1e-3

    cost = pose_graph_error(x, edges, N)
    print(f"iter 0: cost={cost:.6f}")

    for it in range(1, max_iters + 1):
        H, b = build_normal_equations(x, edges, N, lambda_damp=(lam if lm else 0.0))
        dx = spsolve(H, -b)

        step_norm = float(np.linalg.norm(dx))
        if step_norm < tol:
            print(f"iter {it}: step_norm {step_norm:.3e} < tol; stop.")
            break

        x_new = x + dx
        # wrap angles
        for n in range(N):
            x_new[3*n+2] = wrap_angle(x_new[3*n+2])

        new_cost = pose_graph_error(x_new, edges, N)

        if not lm:
            x = x_new
            cost = new_cost
            print(f"iter {it}: cost={cost:.6f}, step={step_norm:.3e}")
            continue

        # LM acceptance rule (simple): accept if cost decreases, otherwise increase damping
        if new_cost < cost:
            x = x_new
            cost = new_cost
            lam = max(lam / 3.0, 1e-9)
            print(f"iter {it}: cost={cost:.6f}, step={step_norm:.3e}, lambda={lam:.2e} (accepted)")
        else:
            lam = min(lam * 5.0, 1e9)
            print(f"iter {it}: cost did not decrease ({new_cost:.6f} >= {cost:.6f}); lambda={lam:.2e} (rejected)")

    return x


def make_information(sig_xy: float, sig_th: float) -> np.ndarray:
    cov = np.diag([sig_xy**2, sig_xy**2, sig_th**2])
    return np.linalg.inv(cov)


def simulate_pose_graph(N: int = 50, seed: int = 7) -> Tuple[np.ndarray, List[EdgeSE2], np.ndarray]:
    """
    Create a synthetic 2D pose graph:
      - ground truth along a gentle curve
      - odometry edges between i and i+1
      - loop closures every ~10 nodes
    """
    rng = np.random.default_rng(seed)

    # ground truth
    gt = np.zeros(3 * N, dtype=float)
    x = np.array([0.0, 0.0, 0.0])
    for i in range(N):
        gt[3*i:3*i+3] = x
        # move forward with slow turn
        u = np.array([0.5, 0.0, 0.03])
        x = se2_compose(x, u)

    edges: List[EdgeSE2] = []

    # odometry
    Om_odo = make_information(sig_xy=0.05, sig_th=0.02)
    for i in range(N - 1):
        xi = gt[3*i:3*i+3]
        xj = gt[3*(i+1):3*(i+1)+3]
        z = se2_between(xi, xj)
        z_noisy = z + rng.normal([0, 0, 0], [0.05, 0.05, 0.02])
        z_noisy[2] = wrap_angle(z_noisy[2])
        edges.append(EdgeSE2(i=i, j=i+1, z=z_noisy, Omega=Om_odo))

    # loop closures
    Om_loop = make_information(sig_xy=0.03, sig_th=0.01)
    for i in range(0, N - 12, 10):
        j = i + 10 + rng.integers(-2, 3)
        xi = gt[3*i:3*i+3]
        xj = gt[3*j:3*j+3]
        z = se2_between(xi, xj)
        z_noisy = z + rng.normal([0, 0, 0], [0.03, 0.03, 0.01])
        z_noisy[2] = wrap_angle(z_noisy[2])
        edges.append(EdgeSE2(i=i, j=j, z=z_noisy, Omega=Om_loop))

    # initial guess from chaining noisy odometry
    x0 = np.zeros_like(gt)
    x = np.array([0.0, 0.0, 0.0])
    x0[0:3] = x
    for i in range(N - 1):
        # take the odometry measurement as motion increment
        z = edges[i].z
        x = se2_compose(x, z)
        x0[3*(i+1):3*(i+1)+3] = x

    return x0, edges, gt


def rmse_poses(x: np.ndarray, gt: np.ndarray, N: int) -> Tuple[float, float]:
    """Compute translational RMSE and angular RMSE."""
    dpos = []
    dang = []
    for i in range(N):
        p = x[3*i:3*i+2]
        pg = gt[3*i:3*i+2]
        dpos.append(np.linalg.norm(p - pg))
        dang.append(abs(wrap_angle(float(x[3*i+2] - gt[3*i+2]))))
    return float(np.sqrt(np.mean(np.square(dpos)))), float(np.sqrt(np.mean(np.square(dang))))


def main():
    N = 60
    x0, edges, gt = simulate_pose_graph(N=N, seed=4)

    rmse0 = rmse_poses(x0, gt, N)
    print(f"Initial RMSE: pos={rmse0[0]:.3f} m, ang={rmse0[1]:.3f} rad")

    x_opt = solve_pose_graph(x0, edges, N, max_iters=25, tol=1e-7, lm=True)

    rmse1 = rmse_poses(x_opt, gt, N)
    print(f"Optimized RMSE: pos={rmse1[0]:.3f} m, ang={rmse1[1]:.3f} rad")

    # Optional: dump to CSV for plotting
    out = np.column_stack([x_opt.reshape(N, 3), gt.reshape(N, 3)])
    np.savetxt("Chapter12_Lesson5_poses.csv", out, delimiter=",",
               header="x,y,theta,x_gt,y_gt,theta_gt", comments="")
    print("Wrote Chapter12_Lesson5_poses.csv")


if __name__ == "__main__":
    main()
