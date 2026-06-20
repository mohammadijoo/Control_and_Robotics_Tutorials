# ===== Code block 1 extracted from Chapter7/Lesson2.html =====
import numpy as np
from scipy.linalg import expm

def forced_response_convolution(A, B, u_fun, t_grid):
    """
    Compute x_zs(t_k) = ∫_0^{t_k} exp(A (t_k - s)) B u(s) ds
    using trapezoidal rule on the provided time grid.
    """
    n = A.shape[0]
    xzs = np.zeros((len(t_grid), n))
    for k, tk in enumerate(t_grid):
        if k == 0:
            xzs[k] = 0.0
            continue
        # trapezoidal weights
        s = t_grid[:k+1]
        us = np.array([u_fun(si) for si in s])  # shape (k+1, m)
        # accumulate integral
        acc = np.zeros((n,))
        for j in range(k):
            sj, sj1 = s[j], s[j+1]
            dt = sj1 - sj
            K0 = expm(A * (tk - sj))  @ B @ us[j]
            K1 = expm(A * (tk - sj1)) @ B @ us[j+1]
            acc += 0.5 * (K0 + K1) * dt
        xzs[k] = acc
    return xzs

def total_state(A, x0, xzs, t_grid):
    X = np.zeros_like(xzs)
    for k, tk in enumerate(t_grid):
        X[k] = expm(A * tk) @ x0 + xzs[k]
    return X

# Example (2-state, 1-input)
A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
B = np.array([[0.0],
              [1.0]])
x0 = np.array([1.0, 0.0])

# Define a piecewise-continuous input u(t)
def u_fun(t):
    # u(t) = 1 for t in [0,2], else 0 (unit pulse)
    return np.array([1.0 if 0.0 <= t <= 2.0 else 0.0])

t_grid = np.linspace(0.0, 6.0, 601)  # dt = 0.01
xzs = forced_response_convolution(A, B, u_fun, t_grid)
x = total_state(A, x0, xzs, t_grid)

print("x(0) =", x[0])
print("x(6) =", x[-1])

# ===== Code block 2 extracted from Chapter7/Lesson2.html =====
import numpy as np
from scipy.signal import StateSpace, lsim

# Build an equivalent state-space model for simulation check
C = np.eye(2)
D = np.zeros((2,1))
sys = StateSpace(A, B, C, D)

# Create input samples on t_grid
u_samples = np.array([u_fun(t)[0] for t in t_grid])

# lsim returns state trajectory if requested (SciPy version dependent)
tout, yout, xout = lsim(sys, U=u_samples, T=t_grid, X0=x0)

# Compare xout (simulation) to x (convolution)
err = np.max(np.linalg.norm(xout - x, axis=1))
print("max ||x_sim - x_conv|| =", err)
