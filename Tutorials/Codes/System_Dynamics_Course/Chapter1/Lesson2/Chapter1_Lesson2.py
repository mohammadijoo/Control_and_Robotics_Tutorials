
import numpy as np

# Discrete-time signals: vectors u of length N
N = 100
n = np.arange(N, dtype=float)

# Example inputs
u1 = np.sin(0.1 * n)        # signal 1
u2 = np.cos(0.07 * n)       # signal 2

# 1) Static linear system: y[k] = 2 * u[k]
def sys_static_linear(u):
    return 2.0 * u

# 2) Static nonlinear system: y[k] = u[k]^2
def sys_static_nonlinear(u):
    return u**2

# 3) Dynamic linear system: first-order difference
#    y[k+1] = a * y[k] + b * u[k], with y[0] = 0
def sys_dynamic_linear(u, a=0.8, b=0.2):
    y = np.zeros_like(u)
    for k in range(N - 1):
        y[k + 1] = a * y[k] + b * u[k]
    return y

# Helper: approximate test of linearity by sampling a1,a2
def check_linearity(sys_fun, u1, u2, alpha=1.3, beta=-0.7):
    y1 = sys_fun(u1)
    y2 = sys_fun(u2)
    lhs = sys_fun(alpha * u1 + beta * u2)
    rhs = alpha * y1 + beta * y2
    error = np.linalg.norm(lhs - rhs) / max(1e-12, np.linalg.norm(rhs))
    return error

# Helper: approximate test of time invariance for shift d
def check_time_invariance(sys_fun, u, d=5):
    # zero-pad left shift (circular effects avoided by padding)
    u_shifted = np.zeros_like(u)
    if d < N:
        u_shifted[d:] = u[:N - d]

    y = sys_fun(u)
    y_shifted_input = sys_fun(u_shifted)

    y_time_shifted = np.zeros_like(y)
    if d < N:
        y_time_shifted[d:] = y[:N - d]

    error = np.linalg.norm(y_shifted_input - y_time_shifted) / max(1e-12, np.linalg.norm(y_time_shifted))
    return error

# Run tests
for name, sys_fun in [
    ("static_linear", sys_static_linear),
    ("static_nonlinear", sys_static_nonlinear),
    ("dynamic_linear", sys_dynamic_linear),
]:
    lin_err = check_linearity(sys_fun, u1, u2)
    ti_err = check_time_invariance(sys_fun, u1)
    print("System:", name)
    print("  linearity error ~", lin_err)
    print("  time-invariance error ~", ti_err)
      