# Chapter12_Lesson2.py
# Minimum-energy steering for continuous-time LTI systems.
# Requires: numpy, scipy
#
# Model:
#   x_dot(t) = A x(t) + B u(t)
# Goal:
#   steer x(0)=x0 to x(T)=xf while minimizing integral_0^T u(t)^T u(t) dt.

import numpy as np
from scipy.linalg import expm
from scipy.integrate import solve_ivp


def controllability_gramian(A: np.ndarray, B: np.ndarray, T: float, N: int = 4001) -> np.ndarray:
    """Finite-horizon controllability Gramian by trapezoidal quadrature.

    Wc(T) = integral_0^T exp(A (T-t)) B B^T exp(A^T (T-t)) dt.
    """
    n = A.shape[0]
    W = np.zeros((n, n), dtype=float)
    ts = np.linspace(0.0, T, N)
    dt = T / (N - 1)

    for k, t in enumerate(ts):
        E = expm(A * (T - t))
        integrand = E @ B @ B.T @ E.T
        weight = 0.5 if k == 0 or k == N - 1 else 1.0
        W += weight * integrand

    return W * dt


def minimum_energy_control(A: np.ndarray, B: np.ndarray, x0: np.ndarray, xf: np.ndarray, T: float):
    """Return u_star(t), Wc(T), lambda, and minimum energy.

    If Wc(T) is nonsingular:
        u_star(t) = B^T exp(A^T (T-t)) Wc(T)^(-1) (xf - exp(A T) x0)
        E_min     = z^T Wc(T)^(-1) z
    """
    W = controllability_gramian(A, B, T)
    z = xf - expm(A * T) @ x0

    # solve(W, z) is numerically better than inv(W) @ z
    lam = np.linalg.solve(W, z)

    def u_star(t: float) -> np.ndarray:
        return B.T @ expm(A.T * (T - t)) @ lam

    E_min = float(z.T @ lam)
    return u_star, W, lam, E_min


def simulate_closed_loop(A: np.ndarray, B: np.ndarray, u_star, x0: np.ndarray, T: float, N: int = 501):
    """Simulate x_dot = A x + B u_star(t)."""
    def rhs(t, x):
        return A @ x + B @ u_star(t)

    sol = solve_ivp(rhs, (0.0, T), x0, t_eval=np.linspace(0.0, T, N), rtol=1e-9, atol=1e-11)
    return sol.t, sol.y.T


if __name__ == "__main__":
    # Double integrator: x1_dot = x2, x2_dot = u.
    A = np.array([[0.0, 1.0],
                  [0.0, 0.0]])
    B = np.array([[0.0],
                  [1.0]])

    T = 2.0
    x0 = np.array([0.0, 0.0])
    xf = np.array([1.0, 0.0])

    u_star, W, lam, E_min = minimum_energy_control(A, B, x0, xf, T)
    t, x = simulate_closed_loop(A, B, u_star, x0, T)

    print("Wc(T) =")
    print(W)
    print("condition number of Wc(T):", np.linalg.cond(W))
    print("lambda = Wc(T)^(-1) z =", lam)
    print("minimum energy =", E_min)
    print("terminal state reached =", x[-1])
    print("target state =", xf)
    print("terminal error norm =", np.linalg.norm(x[-1] - xf))

    # Sample the optimal input.
    for ti in np.linspace(0.0, T, 5):
        print(f"u_star({ti:.2f}) = {u_star(float(ti))[0]: .6f}")
