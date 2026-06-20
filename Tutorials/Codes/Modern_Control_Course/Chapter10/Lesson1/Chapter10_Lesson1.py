"""
Chapter10_Lesson1.py

State steering for a continuous-time double integrator using a
piecewise-constant input sequence.

System:
    x_dot = A x + B u
    A = [[0, 1],
         [0, 0]]
    B = [[0],
         [1]]

The script builds the finite-horizon input-to-final-state map S such that

    x(T) = Phi^N x(0) + S u_sequence

and solves the minimum-norm steering sequence.
"""

import numpy as np
import matplotlib.pyplot as plt

try:
    from scipy.linalg import matrix_power
except Exception:
    matrix_power = np.linalg.matrix_power


def build_double_integrator_maps(T: float, N: int):
    """Return Phi and Gamma for exact zero-order-hold discretization."""
    if T <= 0:
        raise ValueError("T must be positive.")
    if N <= 0:
        raise ValueError("N must be positive.")

    dt = T / N
    Phi = np.array([[1.0, dt], [0.0, 1.0]])
    Gamma = np.array([[0.5 * dt**2], [dt]])
    return Phi, Gamma, dt


def build_steering_matrix(Phi: np.ndarray, Gamma: np.ndarray, N: int):
    """
    Build S = [Phi^(N-1)Gamma, Phi^(N-2)Gamma, ..., Gamma].
    For scalar input, S has shape (state_dimension, N).
    """
    n = Phi.shape[0]
    S = np.zeros((n, N))
    for k in range(N):
        S[:, [k]] = matrix_power(Phi, N - 1 - k) @ Gamma
    return S


def minimum_norm_input(S: np.ndarray, target_shift: np.ndarray):
    """
    Solve S u = target_shift with minimum Euclidean norm:
        u* = S^T (S S^T)^(-1) target_shift
    This formula assumes S has full row rank.
    """
    gram = S @ S.T
    rank = np.linalg.matrix_rank(S)
    if rank < S.shape[0]:
        raise ValueError(
            f"The steering map is rank deficient: rank(S)={rank}, "
            f"state dimension={S.shape[0]}."
        )
    return S.T @ np.linalg.solve(gram, target_shift)


def simulate(Phi: np.ndarray, Gamma: np.ndarray, x0: np.ndarray, u: np.ndarray):
    """Simulate x_{k+1} = Phi x_k + Gamma u_k."""
    x = np.zeros((Phi.shape[0], len(u) + 1))
    x[:, 0] = x0
    for k, uk in enumerate(u):
        x[:, k + 1] = Phi @ x[:, k] + (Gamma[:, 0] * uk)
    return x


def main():
    T = 2.0
    N = 60
    x0 = np.array([0.0, 0.0])
    xf = np.array([1.0, 0.0])

    Phi, Gamma, dt = build_double_integrator_maps(T, N)
    S = build_steering_matrix(Phi, Gamma, N)

    free_response = np.linalg.matrix_power(Phi, N) @ x0
    target_shift = xf - free_response
    u = minimum_norm_input(S, target_shift)

    x = simulate(Phi, Gamma, x0, u)

    print("Rank of finite-horizon steering map S:", np.linalg.matrix_rank(S))
    print("Requested final state:", xf)
    print("Achieved final state:", x[:, -1])
    print("Final steering error norm:", np.linalg.norm(x[:, -1] - xf))
    print("Input energy approximation:", dt * np.sum(u**2))

    time = np.linspace(0.0, T, N + 1)
    input_time = np.linspace(0.0, T - dt, N)

    plt.figure()
    plt.plot(time, x[0, :], label="position")
    plt.plot(time, x[1, :], label="velocity")
    plt.xlabel("time")
    plt.ylabel("state")
    plt.title("State steering: double integrator")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.step(input_time, u, where="post")
    plt.xlabel("time")
    plt.ylabel("input u")
    plt.title("Minimum-norm piecewise-constant steering input")
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()
