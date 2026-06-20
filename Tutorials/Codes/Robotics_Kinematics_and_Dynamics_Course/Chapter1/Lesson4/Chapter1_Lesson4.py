import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Joint parameters
J = 0.01   # inertia
b = 0.1    # viscous damping
k = 1.0    # stiffness

def joint_ode(t, x):
    """
    x[0] = q (angle)
    x[1] = qd (angular velocity)
    """
    q, qd = x
    # Constant step torque input
    u = 1.0
    qdd = (u - b * qd - k * q) / J
    return [qd, qdd]

# High-level integrator (Runge-Kutta) via solve_ivp
t0, tf = 0.0, 5.0
x0 = [0.0, 0.0]  # q(0) = 0, qd(0) = 0

sol = solve_ivp(joint_ode, (t0, tf), x0, max_step=0.01, dense_output=True)

t_grid = np.linspace(t0, tf, 500)
x_grid = sol.sol(t_grid)

plt.figure()
plt.plot(t_grid, x_grid[0], label="q(t)")
plt.plot(t_grid, x_grid[1], label="qd(t)")
plt.xlabel("t [s]")
plt.legend()
plt.title("Single-DOF joint response (Python solve_ivp)")
plt.show()

# From-scratch explicit Euler integrator (for concept illustration)
def euler_integrate(f, t0, tf, x0, h):
    t_values = [t0]
    x_values = [np.array(x0, dtype=float)]
    t = t0
    x = np.array(x0, dtype=float)
    while t < tf:
        dx = np.array(f(t, x))
        x = x + h * dx
        t = t + h
        t_values.append(t)
        x_values.append(x.copy())
    return np.array(t_values), np.vstack(x_values)

t_e, x_e = euler_integrate(joint_ode, 0.0, 5.0, x0, h=0.001)
      
