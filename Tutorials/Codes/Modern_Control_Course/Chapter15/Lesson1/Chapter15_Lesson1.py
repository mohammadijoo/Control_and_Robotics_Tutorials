# Chapter15_Lesson1.py
# Observability Gramian for continuous-time LTI systems.
#
# Required libraries:
#   pip install numpy scipy matplotlib
#
# Optional control library:
#   pip install control
#
# The example uses the zero-input system:
#   x_dot = A x,  y = C x
# and computes:
#   W_o(T) = integral_0^T exp(A^T t) C^T C exp(A t) dt
# and, when A is Hurwitz:
#   A^T W_o + W_o A + C^T C = 0.

import numpy as np
from scipy.linalg import expm, solve_continuous_lyapunov, eigvals
from numpy.linalg import matrix_rank, eigvalsh


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Return O = [C; C A; ...; C A^(n-1)]."""
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def finite_observability_gramian(A: np.ndarray, C: np.ndarray, T: float, steps: int = 5000) -> np.ndarray:
    """
    Numerically approximate W_o(T) using the trapezoidal rule.

    W_o(T) = integral_0^T exp(A^T t) C^T C exp(A t) dt.
    """
    if T <= 0:
        raise ValueError("T must be positive.")
    n = A.shape[0]
    Q = C.T @ C
    ts = np.linspace(0.0, T, steps + 1)
    W = np.zeros((n, n), dtype=float)

    for k, t in enumerate(ts):
        E = expm(A * t)
        integrand = E.T @ Q @ E
        weight = 0.5 if k == 0 or k == steps else 1.0
        W += weight * integrand

    W *= T / steps
    return 0.5 * (W + W.T)


def finite_observability_gramian_ode(A: np.ndarray, C: np.ndarray, T: float) -> np.ndarray:
    """
    Compute W_o(T) through the equivalent matrix differential equation:
      dW/dt = A^T W + W A + C^T C,  W(0)=0.

    For compactness, this routine uses scipy.integrate.solve_ivp.
    """
    from scipy.integrate import solve_ivp

    n = A.shape[0]
    Q = C.T @ C

    def rhs(_, w_flat):
        W = w_flat.reshape(n, n)
        Wdot = A.T @ W + W @ A + Q
        return Wdot.reshape(-1)

    sol = solve_ivp(rhs, (0.0, T), np.zeros(n * n), rtol=1e-10, atol=1e-12)
    W = sol.y[:, -1].reshape(n, n)
    return 0.5 * (W + W.T)


def infinite_observability_gramian(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """
    Compute W_o for Hurwitz A using:
      A^T W_o + W_o A + C^T C = 0.

    SciPy's solve_continuous_lyapunov solves:
      M X + X M^T = Q.
    Choose M=A^T and Q=-(C^T C).
    """
    Q = C.T @ C
    W = solve_continuous_lyapunov(A.T, -Q)
    return 0.5 * (W + W.T)


def output_energy(A: np.ndarray, C: np.ndarray, x0: np.ndarray, T: float) -> float:
    """Compute x0^T W_o(T) x0."""
    W = finite_observability_gramian(A, C, T)
    return float(x0.T @ W @ x0)


def main() -> None:
    # Example 1: observable stable second-order system
    A = np.array([[0.0, 1.0],
                  [-2.0, -3.0]])
    C = np.array([[1.0, 0.0]])

    print("Eigenvalues of A:", eigvals(A))
    O = observability_matrix(A, C)
    print("\nObservability matrix O:")
    print(O)
    print("rank(O) =", matrix_rank(O))

    T = 4.0
    W_T_quad = finite_observability_gramian(A, C, T)
    W_T_ode = finite_observability_gramian_ode(A, C, T)
    W_inf = infinite_observability_gramian(A, C)

    print(f"\nFinite-horizon W_o({T}) by quadrature:")
    print(W_T_quad)
    print("eigenvalues:", eigvalsh(W_T_quad))

    print(f"\nFinite-horizon W_o({T}) by Lyapunov ODE:")
    print(W_T_ode)
    print("difference norm:", np.linalg.norm(W_T_quad - W_T_ode))

    print("\nInfinite-horizon W_o:")
    print(W_inf)
    print("eigenvalues:", eigvalsh(W_inf))

    x0 = np.array([1.0, -0.5])
    print("\nInitial state x0:", x0)
    print("Output energy over [0,T] = x0^T W_o(T) x0 =", output_energy(A, C, x0, T))

    # Example 2: unobservable stable system
    A2 = np.array([[-1.0, 0.0],
                   [0.0, -2.0]])
    C2 = np.array([[1.0, 0.0]])
    W2 = finite_observability_gramian(A2, C2, T)
    print("\nUnobservable example W_o(T):")
    print(W2)
    print("rank(O2) =", matrix_rank(observability_matrix(A2, C2)))
    print("eigenvalues:", eigvalsh(W2))


if __name__ == "__main__":
    main()
