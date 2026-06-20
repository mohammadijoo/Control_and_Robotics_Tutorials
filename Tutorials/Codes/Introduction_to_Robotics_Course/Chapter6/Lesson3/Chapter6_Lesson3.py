import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Parameters (illustrative)
A = 3.0e-3          # piston area [m^2]
beta_e = 1.2e9      # effective bulk modulus [Pa]
Vt = 6.0e-5         # total chamber volume [m^3]
m = 8.0             # load mass [kg]
b = 120.0           # viscous friction [N s/m]
Kq = 2.5e-5         # valve-flow gain [m^3/(s*V)]
Kp = 1.0e-11        # valve pressure gain [m^3/(s*Pa)]
Ct = 5.0e-12        # leakage [m^3/(s*Pa)]

def dynamics(t, xs, u_func, FL_func):
    x, xd, p = xs
    u = u_func(t)
    FL = FL_func(t)

    # Valve flow (linearized)
    q = Kq*u - Kp*p

    # Pressure dynamics
    pd = (beta_e/Vt)*(q - A*xd - Ct*p)

    # Mechanical dynamics
    xdd = (A*p - b*xd - FL)/m

    return [xd, xdd, pd]

# Step input in valve command (e.g., volts)
u_func = lambda t: 2.0 if t >= 0.1 else 0.0
FL_func = lambda t: 0.0

xs0 = [0.0, 0.0, 0.0]  # initial [x, x_dot, p]
sol = solve_ivp(lambda t, xs: dynamics(t, xs, u_func, FL_func),
                [0.0, 1.0], xs0, max_step=1e-3)

t = sol.t
x = sol.y[0]
p = sol.y[2]

plt.figure()
plt.plot(t, x)
plt.xlabel("time [s]")
plt.ylabel("position x [m]")
plt.grid(True)

plt.figure()
plt.plot(t, p)
plt.xlabel("time [s]")
plt.ylabel("load pressure p [Pa]")
plt.grid(True)
plt.show()
