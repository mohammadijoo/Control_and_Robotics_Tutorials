import numpy as np
from scipy.integrate import solve_ivp

def companion_from_ode(a, b=1.0):
    """
    Build companion (phase-variable) matrices for:
        y^(n) + a[n-1] y^(n-1) + ... + a[1] y' + a[0] y = b u
    Input:
        a: array-like length n, ordered [a0, a1, ..., a(n-1)]
        b: scalar
    Returns:
        A (n,n), B (n,1), C (1,n), D (1,1)
    """
    a = np.asarray(a, dtype=float)
    n = a.size
    A = np.zeros((n, n))
    # superdiagonal ones
    for i in range(n - 1):
        A[i, i + 1] = 1.0
    # last row: [-a0, -a1, ..., -a(n-1)]
    A[-1, :] = -a
    B = np.zeros((n, 1))
    B[-1, 0] = float(b)
    C = np.zeros((1, n))
    C[0, 0] = 1.0
    D = np.zeros((1, 1))
    return A, B, C, D

# Example: y'' + 3 y' + 2 y = u
A, B, C, D = companion_from_ode(a=[2.0, 3.0], b=1.0)

def u_of_t(t):
    # Example input: unit step
    return 1.0 if t >= 0.0 else 0.0

def f(t, x):
    x = np.asarray(x).reshape(-1, 1)
    dx = A @ x + B * u_of_t(t)
    return dx.flatten()

x0 = np.array([0.0, 0.0])  # y(0), y'(0)
t_span = (0.0, 5.0)
t_eval = np.linspace(t_span[0], t_span[1], 400)

sol = solve_ivp(f, t_span, x0, t_eval=t_eval, rtol=1e-9, atol=1e-12)
y = (C @ sol.y).flatten()

print("A=\n", A)
print("B=\n", B)
print("y(t) first/last:", y[0], y[-1])

# Optional: if python-control is installed
# from control import ss
# sys = ss(A, B, C, D)
