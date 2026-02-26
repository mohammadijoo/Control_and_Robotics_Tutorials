# Chapter11_Lesson4.py
"""
AMR Course — Chapter 11 (SLAM I — Filter-Based SLAM)
Lesson 4: Data Association Challenges

Implements innovation-based data association for EKF-SLAM:
  - Chi-square gating with Normalized Innovation Squared (NIS)
  - Nearest-Neighbor (NN) association
  - Joint Compatibility Branch-and-Bound (JCBB) for small measurement sets

State mean:
  mu = [x, y, theta, m1x, m1y, ..., mNx, mNy]^T
Measurement:
  z = [range, bearing]^T (bearing relative to robot heading)

Dependencies: numpy (required). SciPy is NOT required.
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict

import numpy as np


# ---------------------------
# Numerics
# ---------------------------
def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def erfinv_winitzki(x: float) -> float:
    x = min(0.999999, max(-0.999999, x))
    a = 0.147
    ln = math.log(1.0 - x * x)
    t = 2.0 / (math.pi * a) + ln / 2.0
    return math.copysign(math.sqrt(math.sqrt(t * t - ln / a) - t), x)


def chi2inv_approx(p: float, dof: int) -> float:
    """
    Chi-square inverse CDF via Wilson–Hilferty approximation.
    Good enough for gating thresholds in demos.
    """
    z = math.sqrt(2.0) * erfinv_winitzki(2.0 * p - 1.0)  # Normal quantile approx
    k = float(dof)
    return k * (1.0 - 2.0 / (9.0 * k) + z * math.sqrt(2.0 / (9.0 * k))) ** 3


# ---------------------------
# EKF-SLAM container (mean + covariance)
# ---------------------------
@dataclass
class EKFSLAMState:
    mu: np.ndarray  # (3+2N,)
    P: np.ndarray   # (3+2N, 3+2N)
    N: int

    @staticmethod
    def demo(pose: np.ndarray, landmarks: np.ndarray, seed: int = 7) -> "EKFSLAMState":
        N = landmarks.shape[0]
        mu = np.concatenate([pose, landmarks.reshape(-1)], axis=0)

        dim = 3 + 2 * N
        rng = np.random.RandomState(seed)
        A = rng.randn(dim, dim)
        P = A @ A.T

        S = np.diag([0.2, 0.2, 0.05] + [0.5] * (2 * N))
        P = S @ (P / np.max(np.diag(P))) @ S
        return EKFSLAMState(mu=mu, P=P, N=N)


# ---------------------------
# Range-bearing model and Jacobian
# ---------------------------
def predict_z(pose: np.ndarray, lm: np.ndarray) -> np.ndarray:
    dx = lm[0] - pose[0]
    dy = lm[1] - pose[1]
    r = math.hypot(dx, dy)
    b = wrap_to_pi(math.atan2(dy, dx) - pose[2])
    return np.array([r, b], dtype=float)


def jacobian_H(pose: np.ndarray, lm: np.ndarray, j: int, N: int) -> np.ndarray:
    dx = lm[0] - pose[0]
    dy = lm[1] - pose[1]
    q = dx * dx + dy * dy
    r = math.sqrt(q)

    H = np.zeros((2, 3 + 2 * N), dtype=float)

    # wrt pose (x,y,theta)
    H[0, 0] = -dx / r
    H[0, 1] = -dy / r
    H[1, 0] = dy / q
    H[1, 1] = -dx / q
    H[1, 2] = -1.0

    # wrt landmark j (mx,my)
    idx = 3 + 2 * j
    H[0, idx + 0] = dx / r
    H[0, idx + 1] = dy / r
    H[1, idx + 0] = -dy / q
    H[1, idx + 1] = dx / q
    return H


def innovation_and_S(st: EKFSLAMState, z: np.ndarray, j: int, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    pose = st.mu[0:3]
    lm = st.mu[3 + 2 * j: 3 + 2 * j + 2]
    zhat = predict_z(pose, lm)

    nu = z - zhat
    nu[1] = wrap_to_pi(float(nu[1]))

    H = jacobian_H(pose, lm, j, st.N)
    S = H @ st.P @ H.T + R
    return nu, S


def nis(nu: np.ndarray, S: np.ndarray) -> float:
    return float(nu.T @ np.linalg.solve(S, nu))


# ---------------------------
# NN association (single measurement)
# ---------------------------
def associate_nn(st: EKFSLAMState, z: np.ndarray, R: np.ndarray, gate_prob: float = 0.99) -> Optional[int]:
    gamma = chi2inv_approx(gate_prob, 2)
    best_j, best_d2 = None, float("inf")

    for j in range(st.N):
        nu, S = innovation_and_S(st, z, j, R)
        d2 = nis(nu, S)
        if d2 <= gamma and d2 < best_d2:
            best_j, best_d2 = j, d2
    return best_j


# ---------------------------
# JCBB (multiple measurements)
# ---------------------------
def joint_compatibility(st: EKFSLAMState,
                        pairs: List[Tuple[int, int]],
                        Z: List[np.ndarray],
                        R: np.ndarray,
                        gate_prob: float = 0.99) -> bool:
    k = len(pairs)
    if k == 0:
        return True

    nus, Hs = [], []
    for (i, j) in pairs:
        pose = st.mu[0:3]
        lm = st.mu[3 + 2 * j: 3 + 2 * j + 2]
        zhat = predict_z(pose, lm)
        nu = Z[i] - zhat
        nu[1] = wrap_to_pi(float(nu[1]))
        H = jacobian_H(pose, lm, j, st.N)
        nus.append(nu)
        Hs.append(H)

    nu_stack = np.concatenate(nus, axis=0)
    H_stack = np.vstack(Hs)
    R_stack = np.kron(np.eye(k), R)
    S = H_stack @ st.P @ H_stack.T + R_stack

    d2 = float(nu_stack.T @ np.linalg.solve(S, nu_stack))
    gamma = chi2inv_approx(gate_prob, 2 * k)
    return d2 <= gamma


def jcbb(st: EKFSLAMState, Z: List[np.ndarray], R: np.ndarray, gate_prob: float = 0.99) -> Dict[int, int]:
    """
    Returns meas_index -> landmark_index for the *largest* jointly compatible set.
    Complexity is exponential in worst case: keep M small (e.g., M<=6).
    """
    M, N = len(Z), st.N
    gamma2 = chi2inv_approx(gate_prob, 2)

    candidates: List[List[int]] = []
    for i in range(M):
        js = []
        for j in range(N):
            nu, S = innovation_and_S(st, Z[i], j, R)
            if nis(nu, S) <= gamma2:
                js.append(j)
        candidates.append(js)

    best: List[Tuple[int, int]] = []

    def rec(i: int, used: set, pairs: List[Tuple[int, int]]) -> None:
        nonlocal best
        if len(pairs) + (M - i) < len(best):
            return
        if i == M:
            if len(pairs) > len(best):
                best = pairs.copy()
            return

        for j in candidates[i]:
            if j in used:
                continue
            new_pairs = pairs + [(i, j)]
            if joint_compatibility(st, new_pairs, Z, R, gate_prob):
                used.add(j)
                rec(i + 1, used, new_pairs)
                used.remove(j)

        rec(i + 1, used, pairs)  # leave measurement i unassigned

    rec(0, set(), [])
    return {i: j for (i, j) in best}


# ---------------------------
# Demo
# ---------------------------
def main() -> None:
    np.set_printoptions(precision=3, suppress=True)

    pose_true = np.array([2.0, 1.0, 0.4])
    lms_true = np.array([[5.0, 2.0],
                         [4.0, -1.5],
                         [1.0, 4.0],
                         [7.0, 5.0],
                         [6.0, -2.0]], dtype=float)

    pose_est = pose_true + np.array([0.1, -0.05, 0.03])
    lms_est = lms_true + np.array([0.2, -0.1])

    st = EKFSLAMState.demo(pose_est, lms_est)

    sigma_r = 0.15
    sigma_b = math.radians(2.0)
    R = np.diag([sigma_r**2, sigma_b**2])

    # Three noisy measurements from landmarks 0,2,4
    rng = np.random.RandomState(1)
    ids = [0, 2, 4]
    Z = []
    for idx in ids:
        z = predict_z(pose_true, lms_true[idx]) + rng.multivariate_normal(np.zeros(2), R)
        z[1] = wrap_to_pi(float(z[1]))
        Z.append(z)

    print("Measurements:")
    for i, z in enumerate(Z):
        print(f"  z[{i}] = {z}")

    print("\nNN associations:")
    for i, z in enumerate(Z):
        print(f"  meas {i} -> {associate_nn(st, z, R, 0.99)}")

    print("\nJCBB associations:")
    print(" ", jcbb(st, Z, R, 0.99))


if __name__ == "__main__":
    main()
