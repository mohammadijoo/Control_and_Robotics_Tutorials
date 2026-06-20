"""
Chapter7_Lesson3.py
Modern Control — Chapter 7 (Solutions of LTI State Equations), Lesson 3
Solutions for Constant, Step, and Polynomial Inputs

This script demonstrates:
  1) Constant input u(t)=u0
  2) Step input u(t)=u0 * 1(t-ts)
  3) Polynomial input u(t)=u0 + u1 t + u2 t^2

We compute closed-form expressions using matrix exponentials and phi-functions.
"""

import numpy as np

try:
    from scipy.linalg import expm
    from scipy.integrate import solve_ivp
except ImportError as e:
    raise SystemExit("This script requires SciPy (scipy.linalg.expm, scipy.integrate.solve_ivp).") from e


def phi_series(Z: np.ndarray, m: int, terms: int = 40) -> np.ndarray:
    """
    Compute phi_m(Z) = sum_{j=0}^∞ Z^j / (j+m)!  via truncated series.
    Works for any square Z (including singular). For larger norms, increase terms
    or use more sophisticated algorithms.
    """
    n = Z.shape[0]
    I = np.eye(n)
    # Start with j=0 term
    fact = math.factorial(m)
    S = I / fact
    Zpow = I.copy()
    for j in range(1, terms):
        Zpow = Zpow @ Z
        denom = math.factorial(j + m)
        S = S + Zpow / denom
    return S


def x_constant_input(A, B, x0, u0, t):
    """
    x(t) = e^{At} x0 + ( ∫_0^t e^{A(t-τ)} B dτ ) u0
         = e^{At} x0 + t * phi_1(A t) B u0
    """
    At = A * t
    E = expm(At)
    Phi1 = phi_series(At, m=1)
    return E @ x0 + (t * (Phi1 @ (B @ u0)))


def x_step_input(A, B, x0, u0, t, ts):
    """
    u(t) = u0 * 1(t-ts)
    x(t) = e^{At} x0                                  , t < ts
         = e^{At} x0 + ∫_{ts}^t e^{A(t-τ)} B u0 dτ     , t >= ts
         = e^{At} x0 + (t-ts) * phi_1(A (t-ts)) B u0   , t >= ts
    """
    if t < ts:
        return expm(A * t) @ x0
    else:
        E = expm(A * t)
        dt = t - ts
        Phi1 = phi_series(A * dt, m=1)
        return E @ x0 + (dt * (Phi1 @ (B @ u0)))


def x_poly_input(A, B, x0, coeffs, t):
    """
    Polynomial input: u(t) = sum_{k=0}^p u_k t^k, where u_k are vectors.
    Closed form:
      x(t) = e^{At} x0 + sum_{k=0}^p t^{k+1} k! * phi_{k+1}(A t) B u_k

    coeffs: list [u0, u1, ..., up], each shape (m,)
    """
    At = A * t
    E = expm(At)
    x = E @ x0
    for k, uk in enumerate(coeffs):
        Phik1 = phi_series(At, m=k+1)  # phi_{k+1}(At)
        x = x + (t ** (k + 1)) * math.factorial(k) * (Phik1 @ (B @ uk))
    return x


def simulate_ivp(A, B, x0, u_fun, t_span, t_eval):
    """
    Numerical verification by solving xdot = A x + B u(t)
    """
    A = np.asarray(A)
    B = np.asarray(B)
    x0 = np.asarray(x0)

    def f(t, x):
        return (A @ x) + (B @ u_fun(t))

    sol = solve_ivp(f, t_span=t_span, y0=x0, t_eval=t_eval, rtol=1e-10, atol=1e-12)
    return sol.t, sol.y.T


def main():
    # Example: 2-state, 1-input system
    A = np.array([[0.0, 1.0],
                  [-2.0, -3.0]])
    B = np.array([[0.0],
                  [1.0]])
    x0 = np.array([1.0, 0.0])

    u0 = np.array([2.0])  # constant / step amplitude

    # Time grid
    t_eval = np.linspace(0.0, 5.0, 251)

    # 1) Constant input
    x_cf = np.vstack([x_constant_input(A, B, x0, u0, t) for t in t_eval])

    t_num, x_num = simulate_ivp(A, B, x0, lambda t: u0, (t_eval[0], t_eval[-1]), t_eval)

    max_err = np.max(np.linalg.norm(x_cf - x_num, axis=1))
    print("Constant input: max ||x_closed - x_numeric|| =", max_err)

    # 2) Step input at ts
    ts = 1.5
    x_step_cf = np.vstack([x_step_input(A, B, x0, u0, t, ts) for t in t_eval])

    def u_step(t):
        return u0 if t >= ts else np.zeros_like(u0)

    t_num2, x_num2 = simulate_ivp(A, B, x0, u_step, (t_eval[0], t_eval[-1]), t_eval)

    max_err2 = np.max(np.linalg.norm(x_step_cf - x_num2, axis=1))
    print("Step input: max ||x_closed - x_numeric|| =", max_err2)

    # 3) Polynomial input u(t) = u0 + u1 t
    u1 = np.array([0.5])
    coeffs = [u0, u1]  # u(t)=u0 + u1 t
    x_poly_cf = np.vstack([x_poly_input(A, B, x0, coeffs, t) for t in t_eval])

    def u_poly(t):
        return u0 + u1 * t

    t_num3, x_num3 = simulate_ivp(A, B, x0, u_poly, (t_eval[0], t_eval[-1]), t_eval)

    max_err3 = np.max(np.linalg.norm(x_poly_cf - x_num3, axis=1))
    print("Polynomial input (affine): max ||x_closed - x_numeric|| =", max_err3)

    # Print a sample
    idx = 100
    print("\nSample at t =", t_eval[idx])
    print("x_const_closed =", x_cf[idx])
    print("x_step_closed  =", x_step_cf[idx])
    print("x_poly_closed  =", x_poly_cf[idx])


if __name__ == "__main__":
    import math
    main()
