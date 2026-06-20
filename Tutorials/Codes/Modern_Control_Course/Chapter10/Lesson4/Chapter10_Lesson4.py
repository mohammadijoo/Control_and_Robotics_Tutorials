"""
Chapter10_Lesson4.py

Physical Interpretation: Actuator Placement and Authority
Modern Control - Chapter 10, Lesson 4

This script compares actuator placements for a two-degree-of-freedom
mass-spring-damper system. It computes:
1. The finite-dimensional reachability matrix [B, AB, ..., A^(n-1)B].
2. The finite-horizon controllability Gramian Wc(T).
3. Directional authority along target state directions.
4. A modal projection score based on left eigenvectors of A.

Dependencies:
    pip install numpy scipy
"""

import numpy as np
from scipy.linalg import expm


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return C = [B, AB, A^2B, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = Ak @ A
    return np.hstack(blocks)


def numerical_gramian(A: np.ndarray, B: np.ndarray, T: float = 5.0, N: int = 1000) -> np.ndarray:
    """
    Approximate Wc(T) = int_0^T exp(A tau) B B^T exp(A^T tau) d tau
    by the trapezoidal rule.
    """
    n = A.shape[0]
    W = np.zeros((n, n), dtype=float)
    dt = T / N

    for k in range(N + 1):
        tau = k * dt
        Phi = expm(A * tau)
        integrand = Phi @ B @ B.T @ Phi.T
        weight = 0.5 if k == 0 or k == N else 1.0
        W += weight * integrand

    return W * dt


def modal_authority(A: np.ndarray, B: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute ||w_i^T B||_2 for each left eigenvector w_i^T of A.
    If this value is near zero, mode i has weak or zero direct modal authority.
    """
    eigvals, V = np.linalg.eig(A)
    Vinv = np.linalg.inv(V)
    scores = np.linalg.norm(Vinv @ B, axis=1)
    return eigvals, scores


def summarize_placement(name: str, A: np.ndarray, B: np.ndarray, target_directions: list[tuple[str, np.ndarray]]) -> None:
    print("\n" + "=" * 72)
    print(f"Actuator placement: {name}")
    print("=" * 72)

    Ctrb = controllability_matrix(A, B)
    rank_C = np.linalg.matrix_rank(Ctrb, tol=1e-9)
    print("Rank of [B, AB, ..., A^(n-1)B]:", rank_C, "out of", A.shape[0])

    W = numerical_gramian(A, B, T=5.0, N=800)
    eigW = np.linalg.eigvalsh(W)
    print("Eigenvalues of finite-horizon Gramian Wc(5):")
    print(np.round(eigW, 8))

    print("Directional authority v^T Wc v for selected target directions:")
    for label, v in target_directions:
        v = v.reshape(-1, 1)
        score = float(v.T @ W @ v)
        print(f"  {label:18s}: {score:.8f}")

    eigvals, scores = modal_authority(A, B)
    print("Modal authority scores ||w_i^T B||_2:")
    for lam, score in zip(eigvals, scores):
        print(f"  lambda={lam.real:+.4f}{lam.imag:+.4f}j, score={score:.8f}")


def main() -> None:
    # Two coupled unit masses with spring coupling and small damping:
    # qdd = -K q - D qdot + G u
    K = np.array([[2.0, -1.0],
                  [-1.0, 2.0]])
    D = np.array([[0.08, 0.00],
                  [0.00, 0.08]])
    M = np.eye(2)

    # State x = [q1, q2, q1dot, q2dot]^T
    Z = np.zeros((2, 2))
    I = np.eye(2)
    A = np.block([[Z, I],
                  [-np.linalg.solve(M, K), -np.linalg.solve(M, D)]])

    # Candidate actuator force-placement matrices G in qdd = ... + M^{-1}G u
    candidates = {
        "force on mass 1 only": np.array([[1.0], [0.0]]),
        "force on mass 2 only": np.array([[0.0], [1.0]]),
        "same force on both masses": np.array([[1.0], [1.0]]),
        "independent forces on both masses": np.eye(2),
    }

    target_directions = [
        ("displace mass 1", np.array([1.0, 0.0, 0.0, 0.0])),
        ("displace mass 2", np.array([0.0, 1.0, 0.0, 0.0])),
        ("relative motion", np.array([1.0, -1.0, 0.0, 0.0]) / np.sqrt(2)),
        ("common motion", np.array([1.0, 1.0, 0.0, 0.0]) / np.sqrt(2)),
    ]

    for name, G in candidates.items():
        B = np.vstack([np.zeros((2, G.shape[1])), np.linalg.solve(M, G)])
        summarize_placement(name, A, B, target_directions)


if __name__ == "__main__":
    main()
