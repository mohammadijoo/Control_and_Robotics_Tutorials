import numpy as np
from scipy.integrate import solve_ivp

# Parameters
M = 2.0            # kg
A = 3.0e-4         # m^2 piston area
V0 = 2.0e-5        # m^3 dead volume
k = 1.2            # polytropic exponent
R = 287.0          # J/(kg K)
T = 293.0          # K
b = 15.0           # N s/m viscous friction
p_atm = 101325.0   # Pa

# Valve/orifice simplified flow model: mdot = Kq*u - Kp*(p - p_atm)
Kq = 1.3e-4        # kg/s per unit input
Kp = 2.0e-9        # kg/(s Pa)

def dynamics(t, s):
    x, v, p = s
    V = V0 + A*x
    u = 1.0 if t >= 0.05 else 0.0  # step command
    mdot = Kq*u - Kp*(p - p_atm)

    # Pressure dynamics
    pdot = (k*R*T/V)*mdot - (k*p/V)*(A*v)

    # Mechanical dynamics
    F = A*p - b*v
    vdot = F/M
    xdot = v
    return [xdot, vdot, pdot]

s0 = [0.0, 0.0, p_atm]  # initial position, velocity, pressure
sol = solve_ivp(dynamics, [0, 0.4], s0, max_step=1e-3)

x = sol.y[0]
v = sol.y[1]
p = sol.y[2]

print("Final position (m):", x[-1])
print("Final pressure (kPa):", p[-1]/1000.0)
