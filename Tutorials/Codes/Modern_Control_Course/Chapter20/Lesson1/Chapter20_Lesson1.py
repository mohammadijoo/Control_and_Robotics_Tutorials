# Chapter20_Lesson1.py
# Minimal realization tests for continuous-time LTI systems.
# Requires: numpy
#
# The transfer matrix is G(s) = C (sI - A)^(-1) B + D.
# A realization is minimal iff (A,B) is reachable and (C,A) is observable.

import numpy as np


def matrix_rank(M, tol=1e-10):
    """Numerical rank using singular values."""
    s = np.linalg.svd(np.asarray(M, dtype=float), compute_uv=False)
    return int(np.sum(s > tol))


def reachability_matrix(A, B):
    """R_n = [B, AB, ..., A^(n-1)B]."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = A @ Ak
    return np.hstack(blocks)


def observability_matrix(A, C):
    """O_n = [C; CA; ...; CA^(n-1)]."""
    A = np.asarray(A, dtype=float)
    C = np.asarray(C, dtype=float)
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def is_reachable(A, B, tol=1e-10):
    A = np.asarray(A, dtype=float)
    return matrix_rank(reachability_matrix(A, B), tol) == A.shape[0]


def is_observable(A, C, tol=1e-10):
    A = np.asarray(A, dtype=float)
    return matrix_rank(observability_matrix(A, C), tol) == A.shape[0]


def is_minimal(A, B, C, tol=1e-10):
    """Kalman minimality test."""
    return is_reachable(A, B, tol) and is_observable(A, C, tol)


def markov_parameters(A, B, C, D, count=8):
    """
    Returns h_0, h_1, ..., h_count where
    h_0 = D and h_k = C A^(k-1) B for k >= 1.
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)
    D = np.asarray(D, dtype=float)
    params = [D]
    Ak = np.eye(A.shape[0])
    for _ in range(1, count + 1):
        params.append(C @ Ak @ B)
        Ak = A @ Ak
    return params


def block_hankel_from_markov(h, rows, cols):
    """
    Builds the block Hankel matrix
    H = [h_{i+j+1}] for i=0,...,rows-1 and j=0,...,cols-1.
    This uses strictly proper Markov parameters h_1, h_2, ...
    """
    return np.block([[h[i + j + 1] for j in range(cols)] for i in range(rows)])


def transfer_value(A, B, C, D, s):
    """Evaluate G(s) at a scalar complex frequency s."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)
    D = np.asarray(D, dtype=float)
    n = A.shape[0]
    return C @ np.linalg.solve(s * np.eye(n) - A, B) + D


def report(name, A, B, C, D):
    R = reachability_matrix(A, B)
    O = observability_matrix(A, C)
    h = markov_parameters(A, B, C, D, count=10)
    H = block_hankel_from_markov(h, rows=3, cols=3)

    print(f"\n{name}")
    print("-" * len(name))
    print("A =\n", np.asarray(A, dtype=float))
    print("B =\n", np.asarray(B, dtype=float))
    print("C =\n", np.asarray(C, dtype=float))
    print("D =\n", np.asarray(D, dtype=float))
    print("rank(R_n) =", matrix_rank(R), "of n =", np.asarray(A).shape[0])
    print("rank(O_n) =", matrix_rank(O), "of n =", np.asarray(A).shape[0])
    print("minimal?  =", is_minimal(A, B, C))
    print("estimated Hankel rank =", matrix_rank(H))


if __name__ == "__main__":
    # Nonminimal realization: one unobservable mode at -1 is present internally,
    # but the transfer function is only G(s) = 1/(s+2).
    A_nonmin = np.array([[-1.0, 0.0],
                         [ 0.0,-2.0]])
    B_nonmin = np.array([[1.0],
                         [1.0]])
    C_nonmin = np.array([[0.0, 1.0]])
    D_nonmin = np.array([[0.0]])

    # Minimal realization of the same external behavior G(s) = 1/(s+2).
    A_min = np.array([[-2.0]])
    B_min = np.array([[1.0]])
    C_min = np.array([[1.0]])
    D_min = np.array([[0.0]])

    report("Two-state nonminimal realization", A_nonmin, B_nonmin, C_nonmin, D_nonmin)
    report("One-state minimal realization", A_min, B_min, C_min, D_min)

    for s in [1.0, 2.0, 3.0]:
        g1 = transfer_value(A_nonmin, B_nonmin, C_nonmin, D_nonmin, s)
        g2 = transfer_value(A_min, B_min, C_min, D_min, s)
        print(f"G_nonminimal({s}) = {g1[0,0]:.6f}, G_minimal({s}) = {g2[0,0]:.6f}")
