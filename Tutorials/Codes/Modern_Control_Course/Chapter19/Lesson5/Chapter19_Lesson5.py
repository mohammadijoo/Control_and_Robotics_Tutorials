# Chapter19_Lesson5.py
# Physical Interpretation of Decomposed Subsystems in Kalman Decomposition
# Libraries: NumPy is used for matrix operations. SciPy / python-control can be
# added for advanced state-space and transfer-function workflows.

import numpy as np


def controllability_matrix(A, B):
    """Return [B, AB, ..., A^(n-1)B]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = Ak @ A
    return np.hstack(blocks)


def observability_matrix(A, C):
    """Return stacked [C; CA; ...; CA^(n-1)]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def euler_simulate(A, B, C, D, u_func, x0, t_final=8.0, dt=0.002):
    """Simple from-scratch forward Euler simulation."""
    t = np.arange(0.0, t_final + dt, dt)
    x = np.zeros((len(t), A.shape[0]))
    y = np.zeros((len(t), C.shape[0]))
    x[0] = x0.reshape(-1)
    for k in range(len(t) - 1):
        u = np.asarray(u_func(t[k])).reshape(B.shape[1], 1)
        xk = x[k].reshape(-1, 1)
        y[k] = (C @ xk + D @ u).reshape(-1)
        dx = A @ xk + B @ u
        x[k + 1] = (xk + dt * dx).reshape(-1)
    u = np.asarray(u_func(t[-1])).reshape(B.shape[1], 1)
    y[-1] = (C @ x[-1].reshape(-1, 1) + D @ u).reshape(-1)
    return t, x, y


def build_kalman_decomposed_example():
    """Create a system already arranged as [co, c_no, no_o, no_no]."""
    A = np.array([
        [0.0,  1.0,  0.0,  0.2,  0.0],
        [-2.0, -3.0, 0.0,  0.0,  0.0],
        [0.0,  0.3, -4.0,  0.1,  0.0],
        [0.0,  0.0,  0.0, -0.5,  0.0],
        [0.0,  0.0,  0.0,  0.0,  0.2]
    ])
    B = np.array([[0.0], [1.0], [1.0], [0.0], [0.0]])
    C = np.array([[1.0, 0.0, 0.0, 1.0, 0.0]])
    D = np.array([[0.0]])
    return A, B, C, D


if __name__ == "__main__":
    A, B, C, D = build_kalman_decomposed_example()
    n = A.shape[0]
    Wc = controllability_matrix(A, B)
    Wo = observability_matrix(A, C)

    print("A =\n", A)
    print("B =\n", B)
    print("C =\n", C)
    print("rank(Wc) =", np.linalg.matrix_rank(Wc), "out of", n)
    print("rank(Wo) =", np.linalg.matrix_rank(Wo), "out of", n)
    print("Expected reachable dimension = 3 = dim(co) + dim(c_no)")
    print("Expected observable dimension = 3 = dim(co) + dim(no_o)")
    print("Minimal input-output order = dim(co) = 2")

    # Zero-initial step response: c_no is internally excited but hidden from y.
    step = lambda t: [1.0]
    t, x, y = euler_simulate(A, B, C, D, step, np.zeros(n), t_final=6.0)
    print("Final zero-initial output under unit step:", y[-1, 0])
    print("Final hidden c_no state under unit step:", x[-1, 2])

    # Initial-condition response: no_o is visible; no_no is invisible.
    x0 = np.array([0.0, 0.0, 0.0, 2.0, 1.0])
    zero = lambda t: [0.0]
    t2, x2, y2 = euler_simulate(A, B, C, D, zero, x0, t_final=6.0)
    print("Initial-condition output y(0) from no_o state:", y2[0, 0])
    print("Final no_no state (can evolve while hidden):", x2[-1, 4])
