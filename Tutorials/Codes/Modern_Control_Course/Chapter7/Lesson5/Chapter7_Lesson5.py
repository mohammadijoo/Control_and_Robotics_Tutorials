"""
Chapter7_Lesson5.py
Modern Control — Chapter 7, Lesson 5: Numerical Simulation of State Equations

This script demonstrates:
1) Continuous-time simulation of x_dot = A x + B u(t) using:
   - fixed-step RK4 (implemented from scratch)
   - SciPy's adaptive RK45 (solve_ivp)
2) Exact discrete-time simulation under Zero-Order Hold (ZOH) using:
   x_{k+1} = A_d x_k + B_d u_k
   where A_d = expm(A h), B_d = \int_0^h expm(A tau) B d tau
   computed via Van Loan's augmented-matrix exponential.

Dependencies:
  numpy, scipy

Optional:
  control (python-control) for state-space helper functions.
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm

def u_of_t(t: float) -> float:
    # Smooth input; for ZOH discretization we sample u_k = u(t_k)
    return float(np.sin(2.0 * t))

def f(t: float, x: np.ndarray, A: np.ndarray, B: np.ndarray) -> np.ndarray:
    return A @ x + B.flatten() * u_of_t(t)

def rk4_step(t: float, x: np.ndarray, h: float, A: np.ndarray, B: np.ndarray) -> np.ndarray:
    k1 = f(t, x, A, B)
    k2 = f(t + 0.5*h, x + 0.5*h*k1, A, B)
    k3 = f(t + 0.5*h, x + 0.5*h*k2, A, B)
    k4 = f(t + h, x + h*k3, A, B)
    return x + (h/6.0) * (k1 + 2*k2 + 2*k3 + k4)

def simulate_rk4(A: np.ndarray, B: np.ndarray, x0: np.ndarray, t0: float, tf: float, h: float):
    n_steps = int(np.floor((tf - t0)/h))
    ts = np.zeros(n_steps + 1)
    xs = np.zeros((n_steps + 1, x0.size))
    ts[0] = t0
    xs[0] = x0
    t = t0
    x = x0.copy()
    for k in range(n_steps):
        x = rk4_step(t, x, h, A, B)
        t = t + h
        ts[k+1] = t
        xs[k+1] = x
    return ts, xs

def van_loan_discretization(A: np.ndarray, B: np.ndarray, h: float):
    """
    Computes (A_d, B_d) for ZOH using Van Loan's method:

      expm( [A  B; 0  0] h ) = [A_d  B_d; 0  I]

    For a continuous-time system x_dot = A x + B u with u constant over [k h, (k+1) h).
    """
    n = A.shape[0]
    m = B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    # bottom-right already zeros
    E = expm(M * h)
    Ad = E[:n, :n]
    Bd = E[:n, n:]
    return Ad, Bd

def simulate_zoh_exact(A: np.ndarray, B: np.ndarray, x0: np.ndarray, t0: float, tf: float, h: float):
    Ad, Bd = van_loan_discretization(A, B, h)
    n_steps = int(np.floor((tf - t0)/h))
    ts = np.zeros(n_steps + 1)
    xs = np.zeros((n_steps + 1, x0.size))
    ts[0] = t0
    xs[0] = x0
    x = x0.copy()
    for k in range(n_steps):
        t = t0 + k*h
        uk = np.array([[u_of_t(t)]])  # ZOH sample
        x = Ad @ x + (Bd @ uk).flatten()
        ts[k+1] = t + h
        xs[k+1] = x
    return ts, xs, Ad, Bd

def simulate_scipy_rk45(A: np.ndarray, B: np.ndarray, x0: np.ndarray, t0: float, tf: float, h_out: float):
    sol = solve_ivp(lambda t, x: f(t, x, A, B), (t0, tf), x0, method="RK45", rtol=1e-9, atol=1e-12)
    # sample on a uniform grid for comparison
    ts = np.arange(t0, tf + 1e-12, h_out)
    xs = np.vstack([np.interp(ts, sol.t, sol.y[i, :]) for i in range(sol.y.shape[0])]).T
    return ts, xs

def main():
    # Example: stable 2-state system
    A = np.array([[0.0, 1.0],
                  [-2.0, -3.0]])
    B = np.array([[0.0],
                  [1.0]])
    x0 = np.array([1.0, 0.0])
    t0, tf = 0.0, 10.0
    h = 0.01

    ts_rk4, xs_rk4 = simulate_rk4(A, B, x0, t0, tf, h)
    ts_zoh, xs_zoh, Ad, Bd = simulate_zoh_exact(A, B, x0, t0, tf, h)
    ts_ref, xs_ref = simulate_scipy_rk45(A, B, x0, t0, tf, h_out=h)

    # Compute errors against reference at grid points
    err_rk4 = np.linalg.norm(xs_rk4 - xs_ref, axis=1)
    err_zoh = np.linalg.norm(xs_zoh - xs_ref, axis=1)

    print("A_d (ZOH exact):\n", Ad)
    print("B_d (ZOH exact):\n", Bd)
    print("Final state RK4: ", xs_rk4[-1])
    print("Final state ZOH: ", xs_zoh[-1])
    print("Final state ref: ", xs_ref[-1])
    print("Max error RK4 vs ref: ", float(np.max(err_rk4)))
    print("Max error ZOH vs ref: ", float(np.max(err_zoh)))

    # Optional: python-control demo if installed
    try:
        import control  # type: ignore
        sysc = control.ss(A, B, np.eye(2), np.zeros((2, 1)))
        t = ts_rk4
        u = np.sin(2.0 * t)
        tout, yout, xout = control.forced_response(sysc, T=t, U=u, X0=x0, return_x=True)
        print("python-control forced_response final state:", xout[:, -1])
    except Exception as e:
        print("python-control not used (optional). Reason:", str(e))

if __name__ == "__main__":
    main()
