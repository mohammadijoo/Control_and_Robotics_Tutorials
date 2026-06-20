"""
Chapter16_Lesson2.py
Difference Equations and Discrete-Time State-Space Models
"""

import numpy as np
import matplotlib.pyplot as plt


def simulate_scalar_difference(a, b, u, y0):
    """
    Simulate y[k+1] = a*y[k] + b*u[k]
    Parameters
    ----------
    a, b : float
    u : array_like, length N
    y0 : float
    Returns
    -------
    y : ndarray, length N+1
        y[0] is the initial condition.
    """
    u = np.asarray(u, dtype=float)
    N = len(u)
    y = np.zeros(N + 1, dtype=float)
    y[0] = y0
    for k in range(N):
        y[k + 1] = a * y[k] + b * u[k]
    return y


def simulate_discrete_state_space(A, B, C, D, u_seq, x0):
    """
    Simulate:
        x[k+1] = A x[k] + B u[k]
        y[k]   = C x[k] + D u[k]
    for scalar input u[k].
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(-1, 1)
    C = np.asarray(C, dtype=float).reshape(1, -1)
    D = float(D)
    x = np.asarray(x0, dtype=float).reshape(-1, 1)

    u_seq = np.asarray(u_seq, dtype=float)
    N = len(u_seq)

    nx = A.shape[0]
    X = np.zeros((N + 1, nx), dtype=float)
    Y = np.zeros(N, dtype=float)

    X[0, :] = x.ravel()
    for k in range(N):
        u = float(u_seq[k])
        yk = (C @ x + D * u).item()
        Y[k] = yk
        x = A @ x + B * u
        X[k + 1, :] = x.ravel()

    return X, Y


def main():
    # Example 1: scalar first-order recursion
    N = 40
    u = np.ones(N)
    a, b = 0.85, 0.2
    y0 = 0.0
    y = simulate_scalar_difference(a, b, u, y0)

    # Example 2: second-order difference equation in companion-form state space
    # y[k+2] - 1.5 y[k+1] + 0.7 y[k] = u[k]
    # Let x[k] = [y[k+1], y[k]]^T
    # Then x[k+1] = [[1.5, -0.7], [1, 0]] x[k] + [[1], [0]] u[k]
    # and y[k] = [0, 1] x[k]
    A = np.array([[1.5, -0.7],
                  [1.0,  0.0]])
    B = np.array([[1.0],
                  [0.0]])
    C = np.array([[0.0, 1.0]])
    D = 0.0

    u2 = np.zeros(N)
    u2[0:10] = 1.0  # finite-duration pulse
    x0 = np.array([0.0, 0.0])

    X, Y = simulate_discrete_state_space(A, B, C, D, u2, x0)

    print("First 10 samples of scalar recursion y[k]:")
    for k in range(10):
        print(f"k={k:2d}, y={y[k]: .6f}")
    print()

    print("First 10 outputs of companion-form model:")
    for k in range(10):
        print(f"k={k:2d}, y={Y[k]: .6f}, x={X[k]}")

    # Plot results
    k1 = np.arange(N + 1)
    plt.figure(figsize=(8, 4))
    plt.stem(k1, y, basefmt=" ", use_line_collection=True)
    plt.xlabel("k")
    plt.ylabel("y[k]")
    plt.title("Scalar Difference Equation: y[k+1] = a y[k] + b u[k]")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    k2 = np.arange(N)
    plt.figure(figsize=(8, 4))
    plt.stem(k2, Y, basefmt=" ", use_line_collection=True)
    plt.xlabel("k")
    plt.ylabel("y[k]")
    plt.title("Discrete-Time State-Space Output (Companion Form)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
