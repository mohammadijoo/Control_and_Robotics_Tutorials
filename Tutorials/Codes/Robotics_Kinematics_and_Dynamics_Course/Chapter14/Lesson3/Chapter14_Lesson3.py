import numpy as np
from math import sin
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Physical parameters
J_l = 0.5   # link inertia
J_m = 0.05  # motor inertia
b_l = 0.02  # link viscous friction
b_m = 0.01  # motor viscous friction
k   = 150.0 # joint stiffness
d   = 0.5   # joint damping
m   = 2.0   # link mass
ell = 0.4   # COM distance
g   = 9.81  # gravity
tau_0 = 1.0 # step torque

def tau_m(t):
    return tau_0  # constant step

def flex_joint_ode(t, x):
    q, qdot, th, thdot = x
    spring = k * (th - q)
    damper = d * (thdot - qdot)
    # Link acceleration
    qddot = (spring + damper - m * g * ell * sin(q) - b_l * qdot) / J_l
    # Motor acceleration
    thddot = (tau_m(t) - spring - damper - b_m * thdot) / J_m
    return [qdot, qddot, thdot, thddot]

x0 = [0.0, 0.0, 0.0, 0.0]  # initial conditions
t_span = (0.0, 2.0)
t_eval = np.linspace(t_span[0], t_span[1], 2001)

sol = solve_ivp(flex_joint_ode, t_span, x0, t_eval=t_eval)

q = sol.y[0, :]
th = sol.y[2, :]

plt.figure()
plt.plot(sol.t, q, label="link q")
plt.plot(sol.t, th, "--", label="motor theta")
plt.xlabel("time [s]")
plt.ylabel("angle [rad]")
plt.legend()
plt.grid(True)
plt.show()
      
