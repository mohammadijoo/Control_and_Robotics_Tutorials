# Chapter29_Lesson1.py
# Linear time-varying (LTV) state-space simulation:
#     x_dot = A(t)x + B(t)u(t)
#     y     = C(t)x + D(t)u(t)
#
# Requirements:
#     pip install numpy scipy matplotlib

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def matrices(t: float):
    """Return A(t), B(t), C(t), D(t) for a second-order LTV oscillator."""
    omega = 1.0 + 0.30 * np.sin(0.70 * t)
    damping = 0.15 + 0.05 * np.cos(0.50 * t)

    A = np.array([
        [0.0, 1.0],
        [-(omega ** 2), -2.0 * damping * omega]
    ], dtype=float)

    B = np.array([
        [0.0],
        [1.0 + 0.20 * np.sin(0.30 * t)]
    ], dtype=float)

    C = np.array([
        [1.0 + 0.10 * np.sin(0.40 * t), 0.0]
    ], dtype=float)

    D = np.array([[0.0]], dtype=float)
    return A, B, C, D


def input_signal(t: float) -> np.ndarray:
    """Scalar input written as a length-1 vector."""
    return np.array([np.sin(1.2 * t)], dtype=float)


def rhs_state(t: float, x: np.ndarray) -> np.ndarray:
    A, B, _, _ = matrices(t)
    u = input_signal(t)
    return A @ x + B @ u


def rhs_augmented_state_transition(t: float, z: np.ndarray) -> np.ndarray:
    """
    Integrate x(t) and Phi(t,t0) together.

    z = [x; vec(Phi)] where Phi is flattened column-wise.
    Phi_dot = A(t) Phi, Phi(t0,t0) = I.
    """
    n = 2
    x = z[:n]
    Phi = z[n:].reshape((n, n), order="F")
    A, B, _, _ = matrices(t)
    u = input_signal(t)

    x_dot = A @ x + B @ u
    Phi_dot = A @ Phi

    return np.concatenate([x_dot, Phi_dot.reshape(n * n, order="F")])


def main():
    t0, tf = 0.0, 20.0
    x0 = np.array([1.0, 0.0], dtype=float)
    n = len(x0)

    # Direct state integration
    sol_x = solve_ivp(rhs_state, (t0, tf), x0, dense_output=True, rtol=1e-9, atol=1e-11)

    # Joint integration of state and state-transition matrix
    Phi0 = np.eye(n)
    z0 = np.concatenate([x0, Phi0.reshape(n * n, order="F")])
    sol_z = solve_ivp(
        rhs_augmented_state_transition,
        (t0, tf),
        z0,
        dense_output=True,
        rtol=1e-9,
        atol=1e-11,
    )

    t_grid = np.linspace(t0, tf, 600)
    x_grid = sol_z.sol(t_grid)[:n, :].T

    y_grid = []
    for t, x in zip(t_grid, x_grid):
        _, _, C, D = matrices(float(t))
        u = input_signal(float(t))
        y_grid.append(float((C @ x + D @ u)[0]))
    y_grid = np.array(y_grid)

    Phi_tf_t0 = sol_z.y[n:, -1].reshape((n, n), order="F")
    print("Phi(tf,t0) =")
    print(Phi_tf_t0)
    print("x(tf) =", sol_z.y[:n, -1])
    print("y(tf) =", y_grid[-1])

    plt.figure()
    plt.plot(t_grid, x_grid[:, 0], label="x1(t)")
    plt.plot(t_grid, x_grid[:, 1], label="x2(t)")
    plt.plot(t_grid, y_grid, label="y(t)", linestyle="--")
    plt.xlabel("time")
    plt.ylabel("response")
    plt.title("Chapter 29 Lesson 1: LTV state-space response")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
