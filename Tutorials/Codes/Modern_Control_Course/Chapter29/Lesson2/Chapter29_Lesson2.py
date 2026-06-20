# Chapter29_Lesson2.py
# Time-varying state transition matrix for x_dot = A(t) x
# Methods:
#   1) solve dPhi/dt = A(t) Phi, Phi(t0,t0)=I
#   2) approximate the Peano-Baker series by recursive quadrature

import numpy as np
from scipy.integrate import solve_ivp


def A(t: float) -> np.ndarray:
    """A smooth 2x2 time-varying state matrix."""
    return np.array(
        [
            [0.0, 1.0],
            [-2.0 - 0.5 * np.sin(t), -0.4 + 0.2 * np.cos(2.0 * t)],
        ],
        dtype=float,
    )


def transition_matrix_ode(t0: float, tf: float, n: int = 2) -> np.ndarray:
    """Compute Phi(tf,t0) by integrating the matrix IVP."""
    def rhs(t, y):
        Phi = y.reshape(n, n)
        dPhi = A(t) @ Phi
        return dPhi.reshape(-1)

    y0 = np.eye(n).reshape(-1)
    sol = solve_ivp(rhs, (t0, tf), y0, rtol=1e-10, atol=1e-12)
    if not sol.success:
        raise RuntimeError(sol.message)
    return sol.y[:, -1].reshape(n, n)


def peano_baker(t0: float, tf: float, order: int = 6, steps: int = 4000) -> np.ndarray:
    """
    Approximate Phi(tf,t0) = I + T1(tf) + T2(tf) + ...
    using the recursion
        T_0(t) = I,
        T_k(t) = integral_{t0}^{t} A(s) T_{k-1}(s) ds.

    The nested time ordering is embedded in the recursion.
    """
    if order < 0:
        raise ValueError("order must be nonnegative")
    if steps < 1:
        raise ValueError("steps must be positive")

    n = 2
    grid = np.linspace(t0, tf, steps + 1)
    h = (tf - t0) / steps

    terms = [[np.zeros((n, n)) for _ in range(steps + 1)] for _ in range(order + 1)]
    for j in range(steps + 1):
        terms[0][j] = np.eye(n)

    for k in range(1, order + 1):
        terms[k][0] = np.zeros((n, n))
        for j in range(1, steps + 1):
            smid = 0.5 * (grid[j - 1] + grid[j])
            # Left endpoint for previous term; midpoint for A(t).
            integrand = A(smid) @ terms[k - 1][j - 1]
            terms[k][j] = terms[k][j - 1] + h * integrand

    Phi = sum(terms[k][-1] for k in range(order + 1))
    return Phi


def check_composition(t0: float, tm: float, tf: float) -> None:
    """Check Phi(tf,t0) = Phi(tf,tm) Phi(tm,t0)."""
    Phi_f0 = transition_matrix_ode(t0, tf)
    Phi_fm = transition_matrix_ode(tm, tf)
    Phi_m0 = transition_matrix_ode(t0, tm)
    err = np.linalg.norm(Phi_f0 - Phi_fm @ Phi_m0, ord="fro")
    print("composition error:", err)


if __name__ == "__main__":
    np.set_printoptions(precision=8, suppress=True)

    t0, tf = 0.0, 6.0
    Phi_ode = transition_matrix_ode(t0, tf)
    print("Phi(tf,t0) from matrix ODE:")
    print(Phi_ode)

    for q in [1, 2, 3, 4, 6, 8]:
        Phi_pb = peano_baker(t0, tf, order=q, steps=5000)
        err = np.linalg.norm(Phi_pb - Phi_ode, ord="fro")
        print(f"Peano-Baker order {q:2d} error: {err:.6e}")

    check_composition(0.0, 2.5, 6.0)
