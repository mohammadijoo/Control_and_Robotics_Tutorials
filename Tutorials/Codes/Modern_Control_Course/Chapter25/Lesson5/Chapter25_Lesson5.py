# Chapter25_Lesson5.py
"""
Structural Constraints: Limited Actuators and Sparse Feedback

This script demonstrates three ideas from Chapter 25, Lesson 5:
1. actuator selection changes the controllability matrix;
2. a sparse feedback mask restricts the admissible K matrix;
3. a constrained feedback gain may stabilize the system, but it generally
   cannot reproduce an arbitrary dense pole-placement design.

Required libraries:
    pip install numpy scipy

Optional modern-control libraries to explore further:
    pip install control slycot
"""

import itertools
import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Return C = [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Ap = np.eye(n)
    for _ in range(n):
        blocks.append(Ap @ B)
        Ap = Ap @ A
    return np.hstack(blocks)


def matrix_rank(M: np.ndarray, tol: float = 1e-9) -> int:
    """Numerical rank through singular values."""
    s = np.linalg.svd(M, compute_uv=False)
    return int(np.sum(s > tol))


def gramian_energy_score(A: np.ndarray, B: np.ndarray) -> float:
    """
    Use a finite-horizon Gramian-like proxy by numerical quadrature.
    This works even when open-loop A is not asymptotically stable.
    Larger trace means more directions are easier to actuate.
    """
    n = A.shape[0]
    T = 5.0
    steps = 400
    dt = T / steps
    W = np.zeros((n, n))
    Phi = np.eye(n)
    # crude Euler approximation of exp(A t) for educational transparency
    for _ in range(steps):
        W += Phi @ B @ B.T @ Phi.T * dt
        Phi = Phi + dt * A @ Phi
    return float(np.trace(W))


def lqr_gain(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Continuous-time LQR gain K = R^(-1) B' P."""
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


def apply_feedback_mask(K_dense: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Project a dense gain to a sparse structural pattern."""
    return K_dense * mask


def simulate_closed_loop(A: np.ndarray, B: np.ndarray, K: np.ndarray, x0: np.ndarray):
    """Simulate x_dot = (A - B K)x."""
    Acl = A - B @ K

    def rhs(t, x):
        return Acl @ x

    return solve_ivp(rhs, (0.0, 10.0), x0, dense_output=True, max_step=0.02)


def main() -> None:
    np.set_printoptions(precision=4, suppress=True)

    # Four-state coupled oscillator-like model.
    A = np.array([
        [0.0,  1.0,  0.0,  0.0],
        [-2.0, -0.25, 0.7,  0.0],
        [0.0,  0.0,  0.0,  1.0],
        [0.6,  0.0, -1.5, -0.20],
    ])

    # Candidate physical actuator columns.
    B_candidates = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.2, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.4, 1.0],
    ])

    n = A.shape[0]
    q = B_candidates.shape[1]

    print("Actuator subset analysis")
    print("------------------------")
    for r in range(1, q + 1):
        for subset in itertools.combinations(range(q), r):
            B = B_candidates[:, subset]
            C = controllability_matrix(A, B)
            rank_C = matrix_rank(C)
            score = gramian_energy_score(A, B)
            print(f"subset={subset}, rank(C)={rank_C}/{n}, trace(W_T)≈{score:.3f}")

    # Select two actuators.
    selected = (0, 2)
    B = B_candidates[:, selected]

    # Dense LQR design.
    Q = np.diag([20.0, 1.0, 20.0, 1.0])
    R = np.diag([0.5, 0.5])
    K_dense = lqr_gain(A, B, Q, R)

    # Sparse communication/measurement pattern:
    # actuator 1 sees states x1 and x2; actuator 2 sees states x3 and x4.
    mask = np.array([
        [1.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 1.0],
    ])
    K_sparse = apply_feedback_mask(K_dense, mask)

    print("\nDense K:")
    print(K_dense)
    print("Sparse projected K:")
    print(K_sparse)

    eig_dense = np.linalg.eigvals(A - B @ K_dense)
    eig_sparse = np.linalg.eigvals(A - B @ K_sparse)
    eig_open = np.linalg.eigvals(A)

    print("\nOpen-loop eigenvalues:", eig_open)
    print("Dense closed-loop eigenvalues:", eig_dense)
    print("Sparse closed-loop eigenvalues:", eig_sparse)

    x0 = np.array([1.0, 0.0, -0.7, 0.2])
    sol_dense = simulate_closed_loop(A, B, K_dense, x0)
    sol_sparse = simulate_closed_loop(A, B, K_sparse, x0)

    print("\nFinal dense state:", sol_dense.y[:, -1])
    print("Final sparse state:", sol_sparse.y[:, -1])
    print("\nInterpretation:")
    print("A sparse mask preserves only allowed feedback links. Even when the pair")
    print("(A,B) is controllable, the masked gain may have weaker pole motion,")
    print("higher residual oscillation, or even fail to stabilize in harder examples.")


if __name__ == "__main__":
    main()
