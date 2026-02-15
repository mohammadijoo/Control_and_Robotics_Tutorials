import numpy as np
from scipy.integrate import solve_ivp

# Parameters
I = 0.2      # arm inertia
b = 0.05     # viscous friction
k = 1.0      # linear stiffness (small-angle approx)

M = 1.0      # cart mass
m = 0.5      # arm mass lumped
ell = 0.6    # COM distance
g = 9.81

# PD controller for q
qd, qdd = 0.0, 0.0
Kp, Kd = 15.0, 3.0

def fixed_base(t, s):
    q, qdot = s
    tau = -Kp*(q-qd) - Kd*(qdot-qdd)
    qddot = (tau - b*qdot - k*q)/I
    return [qdot, qddot]

def moving_base(t, s):
    x, xdot, q, qdot = s
    tau = -Kp*(q-qd) - Kd*qdot
    ub  = 0.0  # unactuated base to show coupling

    # Mass matrix
    M11 = M + m
    M12 = m*ell*np.cos(q)
    M22 = I + m*ell**2
    Mass = np.array([[M11, M12],
                     [M12, M22]])

    # Bias terms
    h1 = -m*ell*np.sin(q)*qdot**2
    h2 =  m*g*ell*np.sin(q)

    acc = np.linalg.solve(Mass, np.array([ub - h1, tau - h2]))
    xddot, qddot = acc
    return [xdot, xddot, qdot, qddot]

# Simulate both systems
tspan = (0, 5)

sol_fixed = solve_ivp(fixed_base, tspan, [0.5, 0.0], max_step=0.01)
sol_move  = solve_ivp(moving_base, tspan, [0.0, 0.0, 0.5, 0.0], max_step=0.01)

print("Fixed-base final q =", sol_fixed.y[0,-1])
print("Moving-base final q =", sol_move.y[2,-1], "final x =", sol_move.y[0,-1])
      