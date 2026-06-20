# Chapter7_Lesson4.py
"""
Modern Control — Chapter 7, Lesson 4
Output Response Using x(t) and y(t) = Cx(t) + Du(t)

This script demonstrates:
1) Analytic output response for constant (step) input using expm(A t)
2) Numeric simulation via SciPy (lsim) when available
3) "From scratch" forward-Euler simulation of xdot = A x + B u, y = C x + D u

Author: (student)
"""

import numpy as np

try:
    from scipy.linalg import expm
    from scipy import signal
    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False


def is_invertible(A: np.ndarray, tol: float = 1e-12) -> bool:
    return np.linalg.cond(A) < 1.0 / tol


def analytic_response_constant_u(A, B, C, D, x0, u0, t_grid):
    """
    For u(t)=u0 (constant), and invertible A:
        x(t) = e^{At} x0 + A^{-1}(e^{At} - I) B u0
        y(t) = C x(t) + D u0
    If A is not invertible or SciPy is unavailable, we fall back to numerical quadrature.
    """
    n = A.shape[0]
    I = np.eye(n)
    y = np.zeros((len(t_grid), C.shape[0]))
    x = np.zeros((len(t_grid), n))

    if SCIPY_AVAILABLE and is_invertible(A):
        Ainv = np.linalg.inv(A)
        for k, t in enumerate(t_grid):
            Phi = expm(A * t)
            xk = Phi @ x0 + (Ainv @ (Phi - I) @ B) @ u0
            yk = (C @ xk) + (D @ u0)
            x[k, :] = xk.ravel()
            y[k, :] = yk.ravel()
        return x, y

    # Fallback: numerical integration of convolution integral using trapezoidal rule
    # x(t) = e^{At} x0 + ∫_0^t e^{A(t-τ)} B u0 dτ
    if not SCIPY_AVAILABLE:
        raise RuntimeError("SciPy not available; cannot compute expm for fallback integration.")

    for k, t in enumerate(t_grid):
        Phi_t = expm(A * t)
        # integrate over τ in [0,t]
        m = max(2, int(400 * t / (t_grid[-1] + 1e-12)))
        tau = np.linspace(0.0, t, m)
        dtau = tau[1] - tau[0]
        integ = np.zeros((n, 1))
        for j, tj in enumerate(tau):
            weight = 0.5 if (j == 0 or j == m - 1) else 1.0
            integ += weight * (expm(A * (t - tj)) @ B) @ u0
        integ *= dtau
        xk = Phi_t @ x0 + integ
        yk = (C @ xk) + (D @ u0)
        x[k, :] = xk.ravel()
        y[k, :] = yk.ravel()

    return x, y


def euler_simulation(A, B, C, D, x0, u_fun, t_grid):
    """
    Forward-Euler simulation (educational; step size affects accuracy):
        x_{k+1} = x_k + h (A x_k + B u(t_k))
        y_k = C x_k + D u(t_k)
    """
    n = A.shape[0]
    p = C.shape[0]
    x = np.zeros((len(t_grid), n))
    y = np.zeros((len(t_grid), p))

    x[0, :] = x0.ravel()
    for k in range(len(t_grid) - 1):
        h = t_grid[k + 1] - t_grid[k]
        uk = u_fun(t_grid[k])
        y[k, :] = (C @ x[k, :].reshape(-1, 1) + D @ uk).ravel()
        xdot = (A @ x[k, :].reshape(-1, 1)) + (B @ uk)
        x[k + 1, :] = (x[k, :].reshape(-1, 1) + h * xdot).ravel()
    # last output
    y[-1, :] = (C @ x[-1, :].reshape(-1, 1) + D @ u_fun(t_grid[-1])).ravel()
    return x, y


def main():
    # Example: 2-state SISO system
    A = np.array([[0.0, 1.0],
                  [-2.0, -3.0]])
    B = np.array([[0.0],
                  [1.0]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.25]])  # include direct feedthrough to show output jump at step changes

    x0 = np.array([[1.0],
                   [0.0]])

    # Step input u(t) = 1 for t >= 0
    u0 = np.array([[1.0]])

    t = np.linspace(0.0, 6.0, 601)

    # Analytic response for constant input
    if SCIPY_AVAILABLE:
        x_a, y_a = analytic_response_constant_u(A, B, C, D, x0, u0, t)
        print("Analytic (constant-input) y(t) at a few times:")
        for ti in [0.0, 0.5, 1.0, 2.0, 6.0]:
            k = int(round(ti / (t[1] - t[0])))
            print(f"t={t[k]:.2f}, y={y_a[k,0]:.6f}")
    else:
        print("SciPy not available: skipping analytic expm-based response.")

    # Numeric simulation with SciPy lsim (if available)
    if SCIPY_AVAILABLE:
        sys = signal.StateSpace(A, B, C, D)
        u = np.ones_like(t)  # step
        tout, y_lsim, x_lsim = signal.lsim(sys, U=u, T=t, X0=x0.ravel())
        print("\nSciPy lsim y(t) at a few times:")
        for ti in [0.0, 0.5, 1.0, 2.0, 6.0]:
            k = int(round(ti / (t[1] - t[0])))
            print(f"t={tout[k]:.2f}, y={y_lsim[k]:.6f}")
    else:
        print("SciPy not available: skipping lsim.")

    # From-scratch Euler
    def u_fun(_t):
        return u0

    x_e, y_e = euler_simulation(A, B, C, D, x0, u_fun, t)
    print("\nEuler (from scratch) y(t) at a few times:")
    for ti in [0.0, 0.5, 1.0, 2.0, 6.0]:
        k = int(round(ti / (t[1] - t[0])))
        print(f"t={t[k]:.2f}, y={y_e[k,0]:.6f}")

    # Optional: save to CSV for plotting elsewhere
    data = np.column_stack([t, y_e[:, 0]])
    np.savetxt("Chapter7_Lesson4_output_euler.csv", data, delimiter=",", header="t,y", comments="")
    print("\nSaved Euler output to Chapter7_Lesson4_output_euler.csv")


if __name__ == "__main__":
    main()
