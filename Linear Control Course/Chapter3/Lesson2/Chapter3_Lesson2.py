import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Physical parameters
m = 1.0   # mass [kg]
b = 0.4   # damping [N s/m]
k = 4.0   # stiffness [N/m]

def msd_ode(t, z):
    """
    z[0] = y (displacement)
    z[1] = v = dy/dt (velocity)
    """
    y, v = z
    F = 1.0  # unit step force for all t >= 0
    a = (F - b * v - k * y) / m
    return [v, a]

# Initial conditions: y(0) = 0, dy/dt(0) = 0
z0 = [0.0, 0.0]

t_span = (0.0, 10.0)
t_eval = np.linspace(t_span[0], t_span[1], 1000)

sol = solve_ivp(msd_ode, t_span, z0, t_eval=t_eval)

t = sol.t
y = sol.y[0, :]

plt.figure()
plt.plot(t, y)
plt.xlabel("t [s]")
plt.ylabel("y(t) [m]")
plt.title("Mass-spring-damper response to unit step force")
plt.grid(True)
plt.show()
