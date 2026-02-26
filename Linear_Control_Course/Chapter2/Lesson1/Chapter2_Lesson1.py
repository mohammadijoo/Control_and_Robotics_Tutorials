import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Physical parameters (e.g., simplified robot joint)
m, b, k = 1.0, 0.2, 1.0  # mass, damping, stiffness

def dynamics(t, x):
    q, qdot = x
    u = 1.0  # unit step input
    qddot = (u - b * qdot - k * q) / m
    return [qdot, qddot]

t_span = (0.0, 10.0)
x0 = [0.0, 0.0]

sol = solve_ivp(dynamics, t_span, x0, max_step=0.01, dense_output=True)

t_plot = np.linspace(0.0, 10.0, 1000)
q_plot = sol.sol(t_plot)[0]

plt.plot(t_plot, q_plot)
plt.xlabel("t (s)")
plt.ylabel("q(t)")
plt.title("Mass-spring-damper step response")
plt.grid(True)
plt.show()

# In robotics, python-control and roboticstoolbox-python use similar ODE models
# to simulate joint and manipulator dynamics before controller design.
