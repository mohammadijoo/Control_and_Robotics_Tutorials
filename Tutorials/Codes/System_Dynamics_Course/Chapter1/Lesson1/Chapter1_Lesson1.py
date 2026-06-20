
import numpy as np
from scipy.integrate import solve_ivp

# Parameters
a = -1.0
b = 1.0

def rhs(t, x):
    """
    Right-hand side of the ODE: dx/dt = a*x + b*u, with u(t) = 1.
    x is a 1D array of length 1 here.
    """
    u = 1.0  # unit-step input
    return a * x + b * u

# Time horizon
t0 = 0.0
tf = 5.0
t_eval = np.linspace(t0, tf, 201)

# Initial condition
x0 = np.array([0.0])

sol = solve_ivp(rhs, (t0, tf), x0, t_eval=t_eval)

# Analytical solution for comparison
x_analytical = 1.0 - np.exp(a * t_eval)  # since a = -1

# Print a few sample points
for t, x_num, x_ana in zip(t_eval[::50], sol.y[0, ::50], x_analytical[::50]):
    print(f"t = {t:.2f}, numerical x = {x_num:.5f}, analytical x = {x_ana:.5f}")
      