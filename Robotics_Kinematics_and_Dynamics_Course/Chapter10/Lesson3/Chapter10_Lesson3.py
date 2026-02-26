import sympy as sp

# Symbols
t = sp.symbols('t')
q = sp.Function('q')(t)          # generalized coordinate q(t)
m, l, g, tau = sp.symbols('m l g tau')

# Kinetic and potential energies
qd = sp.diff(q, t)
T = sp.Rational(1, 2) * m * l**2 * qd**2
V = m * g * l * (1 - sp.cos(q))

L = T - V

# Euler-Lagrange equation: d/dt(dL/dqd) - dL/dq = tau
dL_dq  = sp.diff(L, q)
dL_dqd = sp.diff(L, qd)
d_dt_dL_dqd = sp.diff(dL_dqd, t)

EL_eq = sp.simplify(d_dt_dL_dqd - dL_dq - tau)

print("Euler-Lagrange equation (symbolic):")
print(EL_eq)  # should be m*l**2*qdd + m*g*l*sp.sin(q) - tau

# Replace q''(t) symbolically for clarity
qdd = sp.Function('qdd')(t)
EL_eq_sub = EL_eq.subs(sp.diff(q, (t, 2)), qdd)
print("With qdd substitution:")
print(EL_eq_sub)

# Build a numeric function for simulation using lambdify
q_sym, qd_sym, qdd_sym = sp.symbols('q_sym qd_sym qdd_sym')
EL_numeric = m*l**2 * qdd_sym + m*g*l*sp.sin(q_sym) - tau

rhs_qdd = sp.solve(sp.Eq(EL_numeric, 0), qdd_sym)[0]

# Create a Python function for qdd(q, qd, tau)
qdd_fun = sp.lambdify((q_sym, qd_sym, tau, m, l, g), rhs_qdd, 'numpy')

# Example usage inside an ODE integrator (e.g., scipy.integrate.solve_ivp):
def pendulum_rhs(t, x, m_val, l_val, g_val, tau_fun):
    q_val, qd_val = x
    tau_val = tau_fun(t, q_val, qd_val)  # user-defined torque policy
    qdd_val = qdd_fun(q_val, qd_val, tau_val, m_val, l_val, g_val)
    return [qd_val, qdd_val]
      
