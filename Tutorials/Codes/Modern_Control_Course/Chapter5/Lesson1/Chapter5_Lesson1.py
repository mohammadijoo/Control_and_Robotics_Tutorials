import numpy as np
from dataclasses import dataclass

@dataclass
class StateSpace:
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: np.ndarray

def nth_order_ode_to_ss(a, b=1.0):
    """
    Build state-space matrices for:
        y^(n) + a[n-1] y^(n-1) + ... + a[1] y' + a[0] y = b u

    Parameters
    ----------
    a : array_like, shape (n,)
        Coefficients [a0, a1, ..., a(n-1)].
    b : float
        Input gain.

    Returns
    -------
    StateSpace(A,B,C,D)
    """
    a = np.asarray(a, dtype=float).ravel()
    n = a.size
    A = np.zeros((n, n), dtype=float)
    # shift rows
    for i in range(n - 1):
        A[i, i + 1] = 1.0
    # last row
    A[n - 1, :] = -a  # [-a0, -a1, ..., -a(n-1)]
    B = np.zeros((n, 1), dtype=float)
    B[n - 1, 0] = float(b)
    C = np.zeros((1, n), dtype=float)
    C[0, 0] = 1.0
    D = np.zeros((1, 1), dtype=float)
    return StateSpace(A, B, C, D)

# Example: third-order system: y''' + 3 y'' + 3 y' + 1 y = 2 u
ss = nth_order_ode_to_ss(a=[1.0, 3.0, 3.0], b=2.0)

# Simulate: xdot = A x + B u(t)
from scipy.integrate import solve_ivp

def u_of_t(t):
    # step input
    return 1.0 if t >= 0.0 else 0.0

def f(t, x):
    x = x.reshape(-1, 1)
    dx = ss.A @ x + ss.B * u_of_t(t)
    return dx.ravel()

t0, tf = 0.0, 10.0
x0 = np.array([0.0, 0.0, 0.0])  # y(0), y'(0), y''(0)
sol = solve_ivp(f, (t0, tf), x0, max_step=0.01, rtol=1e-8, atol=1e-10)

# Output y(t) = C x(t)
y = (ss.C @ sol.y).ravel()

print("A=\n", ss.A)
print("B=\n", ss.B)
print("Final y(tf)=", y[-1])

# Optional: python-control wrapper (state-space object)
try:
    import control
    sys = control.ss(ss.A, ss.B, ss.C, ss.D)
    # sys can be used with control.forced_response, control.initial_response, etc.
    print(sys)
except ImportError:
    print("python-control not installed; install with: pip install control")
