# Chapter16_Lesson4.py
# Continuous–Discrete Conversions: Zero-Order Hold, Exact Discretization
# Python implementation (NumPy/SciPy/Matplotlib)

import numpy as np
from scipy.linalg import expm
import matplotlib.pyplot as plt


def exact_discretize(A: np.ndarray, B: np.ndarray, Ts: float):
    """
    Exact ZOH discretization via augmented matrix exponential.
    Works for singular or nonsingular A.
    x[k+1] = Ad x[k] + Bd u[k]
    """
    n, m = B.shape
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    Md = expm(M * Ts)
    Ad = Md[:n, :n]
    Bd = Md[:n, n:]
    return Ad, Bd


def euler_discretize(A: np.ndarray, B: np.ndarray, Ts: float):
    """Forward-Euler (approximate) discretization for comparison."""
    n = A.shape[0]
    Ad = np.eye(n) + Ts * A
    Bd = Ts * B
    return Ad, Bd


def simulate_discrete(Ad, Bd, C, D, u_seq, x0):
    n_steps = len(u_seq)
    x = np.array(x0, dtype=float).reshape(-1, 1)
    ys = []
    xs = [x.flatten()]
    for k in range(n_steps):
        u = np.array([[u_seq[k]]], dtype=float)
        y = C @ x + D @ u
        ys.append(float(y))
        x = Ad @ x + Bd @ u
        xs.append(x.flatten())
    return np.array(xs), np.array(ys)


def main():
    # Example: mass-spring-damper (position/velocity states)
    # m xdd + c xd + k x = b u
    m, c, k, b = 1.0, 0.6, 4.0, 1.0
    A = np.array([[0.0, 1.0],
                  [-k / m, -c / m]])
    B = np.array([[0.0],
                  [b / m]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])

    Ts = 0.1  # sample period [s]

    Ad_exact, Bd_exact = exact_discretize(A, B, Ts)
    Ad_euler, Bd_euler = euler_discretize(A, B, Ts)

    print("A =\n", A)
    print("B =\n", B)
    print("\nExact ZOH discretization:")
    print("Ad_exact =\n", Ad_exact)
    print("Bd_exact =\n", Bd_exact)

    print("\nForward Euler discretization (approximate):")
    print("Ad_euler =\n", Ad_euler)
    print("Bd_euler =\n", Bd_euler)

    # Input sequence: unit step after k=5
    N = 120
    u_seq = np.zeros(N)
    u_seq[5:] = 1.0

    x0 = np.array([0.0, 0.0])

    xs_exact, y_exact = simulate_discrete(Ad_exact, Bd_exact, C, D, u_seq, x0)
    xs_euler, y_euler = simulate_discrete(Ad_euler, Bd_euler, C, D, u_seq, x0)

    t = np.arange(N) * Ts

    plt.figure(figsize=(9, 4.8))
    plt.plot(t, y_exact, label="Exact ZOH")
    plt.plot(t, y_euler, "--", label="Forward Euler")
    plt.step(t, u_seq, where="post", label="Input u[k]", alpha=0.6)
    plt.xlabel("Time [s]")
    plt.ylabel("Output / Input")
    plt.title("Continuous-to-Discrete Conversion: Exact ZOH vs Euler")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # Stability check: map continuous eigenvalues to discrete eigenvalues
    lam_c = np.linalg.eigvals(A)
    lam_d = np.linalg.eigvals(Ad_exact)
    print("\nContinuous eigenvalues:", lam_c)
    print("Discrete eigenvalues (exact):", lam_d)
    print("Magnitudes |lambda_d|:", np.abs(lam_d))


if __name__ == "__main__":
    main()
