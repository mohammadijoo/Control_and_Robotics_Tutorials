# Chapter10_Lesson1.py
# Point-Cloud / Scan Registration Goals — minimal educational reference implementation
#
# This script demonstrates the *goal* of scan registration: estimate the rigid-body transform
# (R, t) that aligns a source point set to a target point set, assuming correspondences are known.
#
# Here we implement a closed-form 2D Procrustes/Kabsch solution + an optional Gauss–Newton refinement
# in SE(2) with fixed correspondences. (ICP variants, correspondence search, and robust kernels are
# treated in later lessons.)

from __future__ import annotations

import numpy as np


def rot2(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def kabsch_2d(P: np.ndarray, Q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Estimate R,t such that Q ~ R P + t in least-squares sense, given paired points.
    P, Q: (N,2)
    Returns: R (2,2), t (2,)
    """
    if P.shape != Q.shape or P.shape[1] != 2:
        raise ValueError("P and Q must have shape (N,2) and match.")

    pbar = P.mean(axis=0)
    qbar = Q.mean(axis=0)

    X = P - pbar
    Y = Q - qbar

    H = X.T @ Y
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Ensure det(R)=+1 (proper rotation)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T

    t = qbar - R @ pbar
    return R, t


def apply_transform(P: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    return (R @ P.T).T + t.reshape(1, 2)


def rmse(P: np.ndarray, Q: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.sum((P - Q) ** 2, axis=1))))


def gn_refine_se2(P: np.ndarray, Q: np.ndarray, R0: np.ndarray, t0: np.ndarray, iters: int = 5) -> tuple[np.ndarray, np.ndarray]:
    """
    Gauss–Newton refinement for SE(2) with fixed correspondences.
    Pose parameterization: (tx, ty, theta).
    """
    # Extract theta from R0
    theta = float(np.arctan2(R0[1, 0], R0[0, 0]))
    tx, ty = float(t0[0]), float(t0[1])

    for _ in range(iters):
        R = rot2(theta)
        t = np.array([tx, ty], dtype=float)

        # residuals r_i = Q_i - (R P_i + t)
        RP = (R @ P.T).T
        r = Q - (RP + t.reshape(1, 2))

        # Jacobian wrt [tx, ty, theta]:
        # dr/dt = -I
        # dr/dtheta = - d(RP)/dtheta
        c, s = np.cos(theta), np.sin(theta)
        dR_dtheta = np.array([[-s, -c], [c, -s]], dtype=float)

        J = np.zeros((2 * P.shape[0], 3), dtype=float)
        b = np.zeros((2 * P.shape[0],), dtype=float)

        for i in range(P.shape[0]):
            d = dR_dtheta @ P[i]
            # r_i is 2-vector
            J[2 * i : 2 * i + 2, 0:2] = -np.eye(2)
            J[2 * i : 2 * i + 2, 2] = -d
            b[2 * i : 2 * i + 2] = r[i]

        # Solve normal equations: (J^T J) delta = J^T b
        H = J.T @ J
        g = J.T @ b
        delta = np.linalg.solve(H, g)

        tx += float(delta[0])
        ty += float(delta[1])
        theta += float(delta[2])

        if np.linalg.norm(delta) < 1e-10:
            break

    return rot2(theta), np.array([tx, ty], dtype=float)


def approx_pose_covariance_se2(P: np.ndarray, theta: float, sigma2: float) -> np.ndarray:
    """
    Approximate covariance of [tx, ty, theta] from linearized least squares with fixed correspondences.
    Sigma ≈ sigma^2 (J^T J)^{-1}
    """
    c, s = np.cos(theta), np.sin(theta)
    dR_dtheta = np.array([[-s, -c], [c, -s]], dtype=float)

    J = np.zeros((2 * P.shape[0], 3), dtype=float)
    for i in range(P.shape[0]):
        d = dR_dtheta @ P[i]
        J[2 * i : 2 * i + 2, 0:2] = -np.eye(2)
        J[2 * i : 2 * i + 2, 2] = -d

    H = J.T @ J
    return sigma2 * np.linalg.inv(H)


def main() -> None:
    np.random.seed(7)

    # Create synthetic "scan" points (source)
    N = 200
    P = np.random.uniform(low=[-5.0, -3.0], high=[5.0, 3.0], size=(N, 2))

    # Ground-truth transform
    theta_gt = np.deg2rad(12.0)
    t_gt = np.array([1.2, -0.7], dtype=float)
    R_gt = rot2(theta_gt)

    # Generate target with noise (paired correspondences)
    Q_clean = apply_transform(P, R_gt, t_gt)
    sigma = 0.02
    Q = Q_clean + np.random.normal(0.0, sigma, size=Q_clean.shape)

    # Closed-form estimate
    R_hat, t_hat = kabsch_2d(P, Q)

    # Optional GN refinement
    R_ref, t_ref = gn_refine_se2(P, Q, R_hat, t_hat, iters=5)

    # Report
    print("Ground truth theta(deg), t:", np.rad2deg(theta_gt), t_gt)
    print("Kabsch      theta(deg), t:", np.rad2deg(np.arctan2(R_hat[1,0], R_hat[0,0])), t_hat)
    print("GN refined  theta(deg), t:", np.rad2deg(np.arctan2(R_ref[1,0], R_ref[0,0])), t_ref)

    Q_hat = apply_transform(P, R_hat, t_hat)
    Q_ref = apply_transform(P, R_ref, t_ref)
    print("RMSE Kabsch  :", rmse(Q_hat, Q))
    print("RMSE Refined :", rmse(Q_ref, Q))

    # Approx covariance (linearized)
    theta_ref = float(np.arctan2(R_ref[1, 0], R_ref[0, 0]))
    Sigma = approx_pose_covariance_se2(P, theta_ref, sigma2=sigma**2)
    print("Approx pose covariance diag [tx,ty,theta]:", np.diag(Sigma))


if __name__ == "__main__":
    main()
