"""
Chapter14_Lesson2.py
PBH Test for Observability for continuous-time LTI systems.

System:
    x_dot = A x + B u
    y     = C x + D u

Observability depends only on (A, C). The PBH test says that (A, C) is
observable iff rank([lambda*I - A; C]) = n for every eigenvalue lambda of A.
"""

from __future__ import annotations

import numpy as np
from numpy.linalg import matrix_rank

try:
    from scipy.linalg import eigvals
except Exception:  # pragma: no cover
    eigvals = None


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Return O = [C; C A; ...; C A^(n-1)]."""
    A = np.asarray(A, dtype=float)
    C = np.atleast_2d(np.asarray(C, dtype=float))
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def pbh_observability_test(A: np.ndarray, C: np.ndarray, tol: float = 1e-9) -> dict:
    """Run the PBH observability test and return a diagnostic dictionary."""
    A = np.asarray(A, dtype=float)
    C = np.atleast_2d(np.asarray(C, dtype=float))

    if A.ndim != 2 or A.shape[0] != A.shape[1]:
        raise ValueError("A must be square.")
    if C.ndim != 2 or C.shape[1] != A.shape[0]:
        raise ValueError("C must have the same number of columns as A has states.")

    n = A.shape[0]
    if eigvals is not None:
        lambdas = eigvals(A)
    else:
        lambdas = np.linalg.eigvals(A)

    rank_O = matrix_rank(observability_matrix(A, C), tol=tol)
    mode_reports = []
    observable = True

    for lam in lambdas:
        pbh_matrix = np.vstack((lam * np.eye(n, dtype=complex) - A, C.astype(complex)))
        r = matrix_rank(pbh_matrix, tol=tol)
        passed = (r == n)
        observable = observable and passed
        mode_reports.append(
            {
                "lambda": lam,
                "rank": int(r),
                "passed": bool(passed),
            }
        )

    return {
        "observable_by_pbh": bool(observable),
        "observability_matrix_rank": int(rank_O),
        "state_dimension": int(n),
        "mode_reports": mode_reports,
    }


def left_eigenvector_check(A: np.ndarray, C: np.ndarray, tol: float = 1e-8) -> list[dict]:
    """
    Check the equivalent condition: no nonzero left eigenvector q* A = lambda q*
    satisfies q* C? For observability, use A.T right eigenvectors v with C v != 0.
    """
    A = np.asarray(A, dtype=float)
    C = np.atleast_2d(np.asarray(C, dtype=float))
    vals, vecs = np.linalg.eig(A.T)
    reports = []
    for k, lam in enumerate(vals):
        v = vecs[:, k]
        cv = C.astype(complex) @ v.astype(complex)
        reports.append(
            {
                "lambda": lam,
                "norm_Cv": float(np.linalg.norm(cv)),
                "visible": bool(np.linalg.norm(cv) > tol),
            }
        )
    return reports


if __name__ == "__main__":
    A1 = np.array([[0.0, 1.0], [-2.0, -3.0]])
    C1 = np.array([[1.0, 0.0]])

    A2 = np.array([[-1.0, 0.0], [0.0, -2.0]])
    C2 = np.array([[1.0, 0.0]])

    for name, A, C in [("Example 1", A1, C1), ("Example 2", A2, C2)]:
        print("=" * 70)
        print(name)
        result = pbh_observability_test(A, C)
        print("PBH observable:", result["observable_by_pbh"])
        print("Rank of observability matrix:", result["observability_matrix_rank"])
        for report in result["mode_reports"]:
            print(
                f"lambda={report['lambda']}, "
                f"rank={report['rank']}, passed={report['passed']}"
            )
        print("Left-eigenvector visibility:")
        for report in left_eigenvector_check(A, C):
            print(
                f"lambda={report['lambda']}, "
                f"||C v||={report['norm_Cv']:.3e}, visible={report['visible']}"
            )
