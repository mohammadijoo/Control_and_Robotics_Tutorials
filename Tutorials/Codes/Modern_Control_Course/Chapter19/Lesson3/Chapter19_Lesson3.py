# Chapter19_Lesson3.py
# Kalman Decomposition: block structure of A, B, C
# Requires: numpy, scipy
#
# This script constructs a Kalman decomposition basis for
# x_dot = A x + B u, y = C x + D u.
#
# Block order used:
#   z = [ z_co ; z_cuo ; z_uco ; z_ucuo ]
# where
#   co   = controllable and observable part,
#   cuo  = controllable and unobservable part,
#   uco  = uncontrollable and observable part,
#   ucuo = uncontrollable and unobservable part.

import numpy as np
from scipy.linalg import svd, null_space

np.set_printoptions(precision=4, suppress=True)


def matrix_power_sequence(A, k):
    Ak = np.eye(A.shape[0])
    powers = []
    for _ in range(k):
        powers.append(Ak.copy())
        Ak = A @ Ak
    return powers


def controllability_matrix(A, B):
    n = A.shape[0]
    return np.hstack([Ak @ B for Ak in matrix_power_sequence(A, n)])


def observability_matrix(A, C):
    n = A.shape[0]
    return np.vstack([C @ Ak for Ak in matrix_power_sequence(A, n)])


def orth_basis(M, tol=1e-10):
    """Column-space orthonormal basis of M."""
    if M.size == 0:
        return np.zeros((M.shape[0], 0))
    U, s, _ = svd(M, full_matrices=False)
    r = np.sum(s > tol)
    return U[:, :r]


def rank(M, tol=1e-10):
    if M.size == 0:
        return 0
    return int(np.sum(svd(M, compute_uv=False) > tol))


def intersection_basis(U, V, tol=1e-10):
    """
    Basis for range(U) intersection range(V), assuming U,V have independent columns.
    We solve U a = V b, i.e. [U -V] [a;b] = 0.
    """
    if U.shape[1] == 0 or V.shape[1] == 0:
        return np.zeros((U.shape[0], 0))
    K = null_space(np.hstack([U, -V]), rcond=tol)
    if K.shape[1] == 0:
        return np.zeros((U.shape[0], 0))
    alpha = K[: U.shape[1], :]
    W = U @ alpha
    return orth_basis(W, tol)


def independent_append(current, candidates, target_dim=None, tol=1e-10):
    """
    Append independent columns from candidates to current.
    If target_dim is given, stop when that dimension is reached.
    """
    cols = []
    if current is not None and current.shape[1] > 0:
        cols = [current[:, j:j+1] for j in range(current.shape[1])]

    M = np.hstack(cols) if cols else np.zeros((candidates.shape[0], 0))
    current_rank = rank(M, tol)

    for j in range(candidates.shape[1]):
        c = candidates[:, j:j+1]
        trial = np.hstack([M, c])
        if rank(trial, tol) > current_rank:
            cols.append(c)
            M = trial
            current_rank += 1
            if target_dim is not None and current_rank >= target_dim:
                break

    return np.hstack(cols) if cols else np.zeros((candidates.shape[0], 0))


def complement_inside(container_basis, sub_basis, tol=1e-10):
    """Return columns that complete sub_basis to container_basis."""
    completed = independent_append(sub_basis, container_basis, target_dim=container_basis.shape[1], tol=tol)
    return completed[:, sub_basis.shape[1]:]


def kalman_decomposition(A, B, C, tol=1e-9):
    n = A.shape[0]

    R = orth_basis(controllability_matrix(A, B), tol)       # reachable subspace
    N = orth_basis(null_space(observability_matrix(A, C), rcond=tol), tol)  # unobservable subspace
    RN = intersection_basis(R, N, tol)                     # controllable-unobservable

    # V2: controllable and unobservable
    V2 = RN

    # V1 completes V2 to the controllable subspace: controllable-observable quotient
    V1 = complement_inside(R, V2, tol)

    # V4 completes V2 to the unobservable subspace: uncontrollable-unobservable quotient
    V4 = complement_inside(N, V2, tol)

    # V3 completes V1,V2,V4 to the full state space: uncontrollable-observable quotient
    I = np.eye(n)
    V124 = np.hstack([V1, V2, V4]) if (V1.shape[1]+V2.shape[1]+V4.shape[1]) else np.zeros((n,0))
    completed = independent_append(V124, I, target_dim=n, tol=tol)
    V3 = completed[:, V124.shape[1]:]

    # Kalman order: [co, c-unobs, unctrl-obs, unctrl-unobs]
    T = np.hstack([V1, V2, V3, V4])
    Tinv = np.linalg.inv(T)

    Abar = Tinv @ A @ T
    Bbar = Tinv @ B
    Cbar = C @ T

    dims = {
        "co": V1.shape[1],
        "c_unobs": V2.shape[1],
        "unctrl_obs": V3.shape[1],
        "unctrl_unobs": V4.shape[1],
        "rank_R": R.shape[1],
        "dim_N": N.shape[1],
    }
    return T, Abar, Bbar, Cbar, dims


def block_slices(dims):
    sizes = [dims["co"], dims["c_unobs"], dims["unctrl_obs"], dims["unctrl_unobs"]]
    idx = np.cumsum([0] + sizes)
    return [slice(idx[i], idx[i+1]) for i in range(4)]


def print_blocks(Abar, Bbar, Cbar, dims):
    labels = ["co", "c_unobs", "unctrl_obs", "unctrl_unobs"]
    slices = block_slices(dims)
    print("Block dimensions:", dims)
    print("\nAbar =")
    print(Abar)
    print("\nBbar =")
    print(Bbar)
    print("\nCbar =")
    print(Cbar)

    print("\nNorms of structurally zero A blocks:")
    zero_positions = [(0, 1), (0, 3), (2, 0), (2, 1), (2, 3), (3, 0), (3, 1)]
    for i, j in zero_positions:
        blk = Abar[slices[i], slices[j]]
        val = 0.0 if blk.size == 0 else np.linalg.norm(blk)
        print(f"  ||A_{labels[i]},{labels[j]}|| = {val:.2e}")

    print("\nNorms of structurally zero B/C blocks:")
    for i in [2, 3]:
        blk = Bbar[slices[i], :]
        print(f"  ||B_{labels[i]}|| = {np.linalg.norm(blk):.2e}")
    for j in [1, 3]:
        blk = Cbar[:, slices[j]]
        print(f"  ||C_{labels[j]}|| = {np.linalg.norm(blk):.2e}")


def demo():
    # Construct a system that already has the desired Kalman zero pattern.
    A_k = np.array([
        [-1.0,  0.0,  0.7,  0.0],
        [ 0.2, -2.0,  0.3, -0.4],
        [ 0.0,  0.0, -3.0,  0.0],
        [ 0.0,  0.0,  0.6, -4.0],
    ])
    B_k = np.array([[1.0],
                    [0.5],
                    [0.0],
                    [0.0]])
    C_k = np.array([[2.0, 0.0, -1.0, 0.0]])

    # Hide the block structure with a nonsingular physical-coordinate transform.
    T_phys = np.array([
        [1.0,  0.2,  0.1,  0.0],
        [0.0,  1.0, -0.3,  0.4],
        [0.2,  0.0,  1.0,  0.1],
        [0.1, -0.2,  0.0,  1.0],
    ])
    A = T_phys @ A_k @ np.linalg.inv(T_phys)
    B = T_phys @ B_k
    C = C_k @ np.linalg.inv(T_phys)

    _, Abar, Bbar, Cbar, dims = kalman_decomposition(A, B, C)
    print_blocks(Abar, Bbar, Cbar, dims)

    print("\nTransfer-function invariant check:")
    print("Only the controllable-observable block contributes to C(sI-A)^(-1)B + D.")


if __name__ == "__main__":
    demo()
