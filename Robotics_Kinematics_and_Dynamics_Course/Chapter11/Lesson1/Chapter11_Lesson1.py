import sympy as sp

# Time variable
t = sp.symbols('t')

# Generalized coordinate and its derivatives
q = sp.Function('q')(t)
qdot  = sp.diff(q, t)
qddot = sp.diff(qdot, t)

# Parameters
m, l, g = sp.symbols('m l g', positive=True)

# Kinetic and potential energy for pendulum
T = sp.Rational(1, 2) * m * l**2 * qdot**2
V = m * g * l * (1 - sp.cos(q))

L = T - V

# Euler-Lagrange equation: d/dt(dL/dqdot) - dL/dq = tau
tau = sp.Function('tau')(t)

dL_dqdot = sp.diff(L, qdot)
d_dt_dL_dqdot = sp.diff(dL_dqdot, t)
dL_dq    = sp.diff(L, q)

EL_eq = sp.simplify(d_dt_dL_dqdot - dL_dq - tau)
print("Euler-Lagrange equation (should be 0):")
print(EL_eq)

# Solve for qddot symbolically
qddot_sol = sp.solve(sp.simplify(EL_eq), qddot)[0]
print("qddot =")
print(sp.simplify(qddot_sol))

# --------------------------------------------------
# Sketch: using roboticstoolbox-python for a general manipulator
# (after previous chapters, students have seen PoE or DH parametrizations)
# --------------------------------------------------
from roboticstoolbox import DHRobot, RevoluteDH

# Simple 1-link pendulum as a DHRobot (for illustration)
link = RevoluteDH(a=0, alpha=0, d=0)
pendulum = DHRobot([link], name="Pendulum")

import numpy as np

def eom_pendulum(q_val, qdot_val, m_val, l_val, g_val):
    # For a generic manipulator, you would normally build the full model with link masses/inertias
    # Using the toolbox, inverse dynamics is:
    M = m_val * l_val**2
    C = 0.0
    G = m_val * g_val * l_val * np.sin(q_val)
    qddot_val = (tau_func(q_val, qdot_val) - C * qdot_val - G) / M
    return qddot_val

# tau_func is a user-defined control torque function
      
