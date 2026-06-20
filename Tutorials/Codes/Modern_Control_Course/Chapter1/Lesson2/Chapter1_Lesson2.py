import numpy as np
import matplotlib.pyplot as plt

# Classical LTI second-order system: y'' + 2*zeta*wn*y' + wn^2 y = wn^2 u
zeta = 0.3
wn = 2.0

# Transfer function representation using python-control
import control as ct

num = [wn**2]
den = [1.0, 2*zeta*wn, wn**2]
G = ct.TransferFunction(num, den)

t = np.linspace(0.0, 10.0, 1000)
t_out, y_step = ct.step_response(G, t)

# Time-varying, saturating version of the same nominal plant
def sat(v, umin=-1.0, umax=1.0):
    return np.maximum(umin, np.minimum(umax, v))

def tv_saturating_ode(t, x, u_cmd):
    # Example: time-varying damping and stiffness
    a1 = 2*zeta*wn * (1.0 + 0.5*np.sin(0.5*t))
    a0 = wn**2 * (1.0 + 0.3*np.cos(0.2*t))

    # Saturated input
    u_phys = sat(u_cmd(t))

    # x = [y, ydot]
    y, ydot = x
    yddot = -a1*ydot - a0*y + wn**2*u_phys
    return np.array([ydot, yddot])

# Simulate the time-varying system using solve_ivp
from scipy.integrate import solve_ivp

def step_input(t):
    return 2.0  # command exceeds actuator limits

def simulate_tv_system(x0):
    sol = solve_ivp(
        fun=lambda tt, xx: tv_saturating_ode(tt, xx, step_input),
        t_span=(0.0, 10.0),
        y0=x0,
        t_eval=t
    )
    return sol.t, sol.y[0]

t_tv, y_tv = simulate_tv_system(x0=[0.0, 0.0])

plt.figure()
plt.plot(t_out, y_step, label="LTI transfer function step response")
plt.plot(t_tv, y_tv, linestyle="--", label="Time-varying, saturating plant")
plt.xlabel("t")
plt.ylabel("y(t)")
plt.legend()
plt.grid(True)
plt.show()
      
