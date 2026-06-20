# Chapter10_Lesson2.py
# Reachable States and Reachable Subspace for continuous-time LTI systems
#
# Model:
#   x_dot(t) = A x(t) + B u(t)
#   x(0) = 0
#   x(T) = integral_0^T exp(A(T-tau)) B u(tau) d tau
#
# This script computes:
#   1) the algebraic reachable subspace span{B, AB, ..., A^(n-1)B}
#   2) a finite-time numerical input-to-state map using piecewise-constant inputs
#   3) the projection of a desired target state onto the reachable subspace

import numpy as np
from scipy.linalg import expm, svd, null_space
from scipy.integrate import quad_vec


def reachability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Build [B, AB, A^2B, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = Ak @ A
    return np.hstack(blocks)


def orthonormal_column_basis(M: np.ndarray, tol: float = 1e-10) -> np.ndarray:
    """Return an orthonormal basis for the column space of M using SVD."""
    U, s, _ = svd(M, full_matrices=False)
    r = int(np.sum(s > tol))
    return U[:, :r]


def project_onto_subspace(x: np.ndarray, Q: np.ndarray) -> np.ndarray:
    """Project x onto span(Q), where Q has orthonormal columns."""
    return Q @ (Q.T @ x)


def finite_time_input_map(A: np.ndarray, B: np.ndarray, T: float, N: int) -> np.ndarray:
    """
    Construct the matrix G_N mapping piecewise-constant controls to x(T).

    If u(t) = u_j on interval [t_j, t_{j+1}), then:
        x(T) ≈ sum_j integral_{t_j}^{t_{j+1}} exp(A(T-tau))B d tau * u_j

    G_N has shape n x (mN).
    """
    n, m = B.shape
    grid = np.linspace(0.0, T, N + 1)
    blocks = []
    for j in range(N):
        a, b = grid[j], grid[j + 1]

        def integrand(tau):
            return expm(A * (T - tau)) @ B

        block, _ = quad_vec(integrand, a, b)
        blocks.append(block.reshape(n, m))
    return np.hstack(blocks)


def least_squares_piecewise_control(G: np.ndarray, x_target: np.ndarray) -> np.ndarray:
    """
    Minimum-norm vector of piecewise-constant control samples that best reaches x_target.
    """
    u_star, *_ = np.linalg.lstsq(G, x_target, rcond=None)
    return u_star


def demo():
    # Third state is dynamically decoupled from the actuator, so it is not reachable.
    A = np.array([
        [0.0, 1.0, 0.0],
        [-2.0, -3.0, 0.0],
        [0.0, 0.0, -1.0]
    ])
    B = np.array([
        [0.0],
        [1.0],
        [0.0]
    ])

    R = reachability_matrix(A, B)
    Q = orthonormal_column_basis(R)

    print("A =\n", A)
    print("B =\n", B)
    print("\nReachability matrix [B AB A^2B] =\n", R)
    print("Dimension of reachable subspace:", Q.shape[1])
    print("Orthonormal basis for reachable subspace =\n", Q)

    x_desired = np.array([1.0, -0.5, 2.0])
    x_reachable_part = project_onto_subspace(x_desired, Q)
    x_unreachable_part = x_desired - x_reachable_part

    print("\nDesired target:", x_desired)
    print("Reachable component:", x_reachable_part)
    print("Unreachable component:", x_unreachable_part)

    T = 3.0
    N = 60
    G = finite_time_input_map(A, B, T, N)
    u_piecewise = least_squares_piecewise_control(G, x_desired)
    xT = G @ u_piecewise

    print("\nFinite-time map rank:", np.linalg.matrix_rank(G))
    print("Reached final state using least-squares input:", xT)
    print("Final error:", x_desired - xT)

    # Left annihilator of the reachable subspace: any vector here cannot be influenced by u.
    left_annihilator = null_space(R.T)
    print("\nBasis for orthogonal complement of reachable subspace =\n", left_annihilator)


if __name__ == "__main__":
    demo()
