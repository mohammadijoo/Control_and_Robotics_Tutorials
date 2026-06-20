"""
Chapter9_Lesson4.py

Internal stability versus external input-output behavior for a nonminimal
continuous-time LTI state-space model.

Model:
    x_dot = A x + B u
    y     = C x + D u

A has one stable mode and one unstable hidden mode:
    A = diag(-1, +1), B = [1, 0]^T, C = [1, 0], D = 0

The transfer function from u to y is G(s)=1/(s+1), which is BIBO stable.
However, the internal state x2(t)=exp(t)x2(0) is unstable if x2(0) != 0.
"""

import math
import numpy as np


A = np.array([[-1.0, 0.0],
              [ 0.0, 1.0]])
B = np.array([[1.0],
              [0.0]])
C = np.array([[1.0, 0.0]])
D = np.array([[0.0]])


def f(x: np.ndarray, u: float) -> np.ndarray:
    """State derivative for the 2-state system."""
    return A @ x + B.flatten() * u


def rk4_step(x: np.ndarray, u: float, h: float) -> np.ndarray:
    """One fixed-step RK4 integration step."""
    k1 = f(x, u)
    k2 = f(x + 0.5 * h * k1, u)
    k3 = f(x + 0.5 * h * k2, u)
    k4 = f(x + h * k3, u)
    return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def simulate(x0: np.ndarray, u_fun, t_final: float = 5.0, h: float = 0.01):
    """Simulate x_dot = Ax + Bu and y = Cx + Du."""
    n_steps = int(t_final / h)
    t = np.linspace(0.0, t_final, n_steps + 1)
    x = np.zeros((n_steps + 1, 2))
    y = np.zeros(n_steps + 1)
    u_hist = np.zeros(n_steps + 1)

    x[0, :] = x0
    for k in range(n_steps):
        u = float(u_fun(t[k]))
        u_hist[k] = u
        y[k] = float(C @ x[k, :].reshape(-1, 1) + D * u)
        x[k + 1, :] = rk4_step(x[k, :], u, h)

    u_hist[-1] = float(u_fun(t[-1]))
    y[-1] = float(C @ x[-1, :].reshape(-1, 1) + D * u_hist[-1])
    return t, u_hist, x, y


def classify_internal_stability(A_matrix: np.ndarray) -> str:
    """Classify continuous-time internal stability by eigenvalues of A."""
    eigenvalues = np.linalg.eigvals(A_matrix)
    if np.all(np.real(eigenvalues) < 0.0):
        return "asymptotically internally stable"
    if np.any(np.real(eigenvalues) > 0.0):
        return "internally unstable"
    return "marginal or inconclusive from this simple test"


def main():
    print("A eigenvalues:", np.linalg.eigvals(A))
    print("Internal classification:", classify_internal_stability(A))
    print("External transfer function from u to y: G(s) = 1/(s + 1)")
    print("External pole: -1, so the input-output map is stable for zero initial state.")

    # Case 1: zero input, hidden unstable initial condition.
    # Output remains zero because C does not measure x2.
    t, u_hist, x, y = simulate(np.array([0.0, 1.0]), lambda tau: 0.0)
    print("\nCase 1: u(t)=0, x(0)=[0, 1]^T")
    print("  final state x(T):", x[-1, :])
    print("  final output y(T):", y[-1])
    print("  max |y(t)|:", np.max(np.abs(y)))
    print("  max ||x(t)||:", np.max(np.linalg.norm(x, axis=1)))

    # Case 2: bounded step input, zero initial state.
    # Transfer behavior is the stable first-order response y(t)=1-exp(-t).
    t, u_hist, x, y = simulate(np.array([0.0, 0.0]), lambda tau: 1.0)
    print("\nCase 2: u(t)=1, x(0)=[0, 0]^T")
    print("  final state x(T):", x[-1, :])
    print("  final output y(T):", y[-1])
    print("  expected y(T) approximately:", 1.0 - math.exp(-t[-1]))
    print("  max |u(t)|:", np.max(np.abs(u_hist)))
    print("  max |y(t)|:", np.max(np.abs(y)))

    try:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(t, y, label="output y(t)")
        plt.plot(t, x[:, 0], "--", label="state x1(t)")
        plt.plot(t, x[:, 1], ":", label="hidden state x2(t)")
        plt.xlabel("time")
        plt.ylabel("response")
        plt.title("Externally stable step response with zero initial hidden state")
        plt.grid(True)
        plt.legend()
        plt.show()
    except Exception:
        pass


if __name__ == "__main__":
    main()
