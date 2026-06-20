"""
Chapter24_Lesson5.py
Modern Control - Chapter 24, Lesson 5
Design examples for multi-input pole placement.

Libraries:
    numpy, scipy, matplotlib
Install:
    pip install numpy scipy matplotlib
"""

import numpy as np
from numpy.linalg import inv, matrix_rank, eig
from scipy.linalg import null_space
from scipy.signal import place_poles
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

np.set_printoptions(precision=5, suppress=True)


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Kalman controllability matrix [B AB ... A^(n-1)B]."""
    n = A.shape[0]
    blocks = [B]
    Ap = np.eye(n)
    for _ in range(1, n):
        Ap = Ap @ A
        blocks.append(Ap @ B)
    return np.hstack(blocks)


def eigenstructure_assignment(A: np.ndarray, B: np.ndarray, poles, seed: int = 24):
    """
    Multi-input pole assignment using eigenvector selection.

    For u = -Kx, desired pairs (lambda_i, v_i) must satisfy
        (A - B K) v_i = lambda_i v_i.
    Put f_i = K v_i. Then
        [A - lambda_i I, -B] [v_i; f_i] = 0.
    After selecting n linearly independent v_i, K = F V^{-1}.
    """
    rng = np.random.default_rng(seed)
    n, m = B.shape
    V = np.zeros((n, n), dtype=float)
    F = np.zeros((m, n), dtype=float)

    for i, lam in enumerate(poles):
        M = np.hstack((A - lam * np.eye(n), -B))
        N = null_space(M)  # basis for [v; f]
        if N.shape[1] == 0:
            raise RuntimeError(f"No nullspace found for lambda={lam}.")

        # Try deterministic/random combinations until V remains well-conditioned.
        best_vec = None
        best_score = -np.inf
        for trial in range(200):
            q = np.ones(N.shape[1]) if trial == 0 else rng.normal(size=N.shape[1])
            s = N @ q
            v = np.real_if_close(s[:n]).astype(float)
            f = np.real_if_close(s[n:]).astype(float)
            V_trial = V.copy()
            V_trial[:, i] = v
            score = matrix_rank(V_trial[:, : i + 1])
            norm_score = np.linalg.norm(v)
            if score + 1e-4 * norm_score > best_score:
                best_score = score + 1e-4 * norm_score
                best_vec = (v, f)
            if score == i + 1:
                best_vec = (v, f)
                break

        V[:, i], F[:, i] = best_vec

    if matrix_rank(V) < n:
        raise RuntimeError("Selected eigenvectors are not linearly independent.")
    K = F @ inv(V)
    return K, V, F


def simulate_closed_loop(A: np.ndarray, B: np.ndarray, K: np.ndarray, x0: np.ndarray):
    """Simulate xdot = (A - B K)x."""
    Acl = A - B @ K

    def rhs(t, x):
        return Acl @ x

    sol = solve_ivp(rhs, (0.0, 8.0), x0, max_step=0.01, dense_output=True)
    return sol


def main():
    # Example: fourth-order plant with two independent actuators.
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [-2.0, -5.0, -4.0, -1.0],
    ])
    B = np.array([
        [0.0, 0.0],
        [1.0, 0.0],
        [0.0, 0.0],
        [0.0, 1.0],
    ])
    desired_poles = np.array([-1.0, -2.0, -3.0, -4.0])

    Ctrb = controllability_matrix(A, B)
    print("rank(C) =", matrix_rank(Ctrb), "out of", A.shape[0])

    # Library design: SciPy implements robust MIMO algorithms.
    scipy_result = place_poles(A, B, desired_poles, method="YT")
    K_scipy = scipy_result.gain_matrix
    print("\nSciPy gain K:")
    print(K_scipy)
    print("Closed-loop eigenvalues (SciPy):", np.sort(eig(A - B @ K_scipy)[0]))

    # From-scratch eigenstructure design.
    K_eig, V, F = eigenstructure_assignment(A, B, desired_poles)
    print("\nEigenstructure-assignment gain K:")
    print(K_eig)
    print("Closed-loop eigenvalues (from scratch):", np.sort(eig(A - B @ K_eig)[0]))
    print("cond(V) =", np.linalg.cond(V))

    # Time-domain check.
    x0 = np.array([1.0, -0.5, 0.8, 0.0])
    sol = simulate_closed_loop(A, B, K_eig, x0)

    plt.figure(figsize=(8, 5))
    for i in range(A.shape[0]):
        plt.plot(sol.t, sol.y[i], label=f"x{i+1}")
    plt.xlabel("time (s)")
    plt.ylabel("state")
    plt.title("Closed-loop response with multi-input pole placement")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
