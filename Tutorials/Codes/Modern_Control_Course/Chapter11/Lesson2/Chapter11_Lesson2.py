# Chapter11_Lesson2.py
"""
PBH (Popov-Belevitch-Hautus) controllability test for continuous-time LTI systems.
The pair (A, B) is controllable iff rank([lambda I - A, B]) = n
for every eigenvalue lambda of A.
"""

import numpy as np


def unique_complex_values(values, tol=1e-8):
    """Cluster numerically repeated eigenvalues."""
    unique = []
    for value in values:
        if not any(abs(value - old) <= tol for old in unique):
            unique.append(value)
    return unique


def numerical_rank(M, tol=1e-9):
    """SVD-based numerical rank for real or complex matrices."""
    s = np.linalg.svd(M, compute_uv=False)
    return int(np.sum(s > tol)), s


def controllability_matrix(A, B):
    """Kalman controllability matrix [B, AB, ..., A^(n-1)B]."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = Ak @ A
    return np.hstack(blocks)


def pbh_rank_test(A, B, tol=1e-9, eig_cluster_tol=1e-8):
    """Return PBH controllability verdict and per-eigenvalue diagnostics."""
    A = np.asarray(A, dtype=complex)
    B = np.asarray(B, dtype=complex)
    n = A.shape[0]
    if A.shape != (n, n):
        raise ValueError("A must be square")
    if B.shape[0] != n:
        raise ValueError("B must have the same number of rows as A")

    eigenvalues = unique_complex_values(np.linalg.eigvals(A), eig_cluster_tol)
    details = []
    ok = True
    for lam in eigenvalues:
        M = np.hstack((lam * np.eye(n, dtype=complex) - A, B))
        rank, singular_values = numerical_rank(M, tol)
        passed = (rank == n)
        ok = ok and passed
        details.append({
            "lambda": lam,
            "rank": rank,
            "required_rank": n,
            "passed": passed,
            "singular_values": singular_values,
        })
    return ok, details


def print_pbh_report(A, B, name):
    print(f"\n{name}")
    print("A =\n", np.array(A, dtype=float))
    print("B =\n", np.array(B, dtype=float))

    Ctrb = controllability_matrix(A, B)
    kalman_rank, kalman_s = numerical_rank(Ctrb)
    print("Kalman matrix rank:", kalman_rank)
    print("Kalman singular values:", kalman_s)

    ok, details = pbh_rank_test(A, B)
    print("PBH controllable:", ok)
    for d in details:
        print(
            f"  lambda={d['lambda']:.6g}, "
            f"rank={d['rank']}/{d['required_rank']}, "
            f"passed={d['passed']}, "
            f"singular_values={np.round(d['singular_values'], 6)}"
        )


if __name__ == "__main__":
    # Example 1: diagonal system with an unactuated mode at lambda = 0.
    A1 = np.diag([0.0, -1.0, -2.0])
    B1 = np.array([[0.0], [1.0], [1.0]])
    print_pbh_report(A1, B1, "Example 1: uncontrollable because the first mode is not actuated")

    # Example 2: same A, but every distinct eigenmode is directly reached.
    B2 = np.array([[1.0], [1.0], [1.0]])
    print_pbh_report(A1, B2, "Example 2: controllable diagonal system")

    # Example 3: repeated eigenvalue with one input is not enough to cover the full eigenspace.
    A3 = np.diag([1.0, 1.0, 2.0])
    B3 = np.array([[1.0], [1.0], [1.0]])
    print_pbh_report(A3, B3, "Example 3: uncontrollable repeated-eigenvalue eigenspace")

    # Example 4: two inputs cover the repeated eigenspace at lambda = 1.
    B4 = np.array([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]])
    print_pbh_report(A3, B4, "Example 4: controllable repeated-eigenvalue case with enough input directions")
