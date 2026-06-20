import sympy as sp
import numpy as np

# --- Symbolic model definition ---

# Generalized coordinates and velocities
x_b, y_b, th_b, q1, q2 = sp.symbols("x_b y_b th_b q1 q2", real=True)
dx_b, dy_b, dth_b, dq1, dq2 = sp.symbols("dx_b dy_b dth_b dq1 dq2", real=True)

q = sp.Matrix([x_b, y_b, th_b, q1, q2])
dq = sp.Matrix([dx_b, dy_b, dth_b, dq1, dq2])

# Parameters
m_b, m1, m2 = sp.symbols("m_b m1 m2", positive=True)
I_b, I1, I2 = sp.symbols("I_b I1 I2", positive=True)
l1, l2, c1, c2 = sp.symbols("l1 l2 c1 c2", positive=True)
g = sp.symbols("g", positive=True)

def R(theta):
    return sp.Matrix([[sp.cos(theta), -sp.sin(theta)],
                      [sp.sin(theta),  sp.cos(theta)]])

# Orientations
th1 = th_b + q1
th2 = th_b + q1 + q2

# Positions
p_b = sp.Matrix([x_b, y_b])
p_c1 = p_b + R(th1) * sp.Matrix([c1, 0])
p_knee = p_b + R(th1) * sp.Matrix([l1, 0])
p_c2 = p_knee + R(th2) * sp.Matrix([c2, 0])
p_f = p_knee + R(th2) * sp.Matrix([l2, 0])  # foot position

# Jacobians for COMs and foot
J_b   = p_b.jacobian(q)
J_c1  = p_c1.jacobian(q)
J_c2  = p_c2.jacobian(q)
J_cf  = p_f.jacobian(q)   # contact Jacobian (position)

# Velocities from Jacobians
v_b  = J_b  * dq
v_c1 = J_c1 * dq
v_c2 = J_c2 * dq

# Angular velocities (planar)
w_b  = dth_b
w_1  = dth_b + dq1
w_2  = dth_b + dq1 + dq2

# Kinetic and potential energy
K = (sp.Rational(1, 2) * m_b * v_b.dot(v_b)
     + sp.Rational(1, 2) * m1 * v_c1.dot(v_c1)
     + sp.Rational(1, 2) * m2 * v_c2.dot(v_c2)
     + sp.Rational(1, 2) * I_b * w_b**2
     + sp.Rational(1, 2) * I1 * w_1**2
     + sp.Rational(1, 2) * I2 * w_2**2)

P = m_b * g * y_b + m1 * g * p_c1[1] + m2 * g * p_c2[1]
L = K - P

# Mass matrix via quadratic form in dq
M_sym = sp.hessian(K, dq)

# Gravity term from potential
g_sym = sp.Matrix([sp.diff(P, qi) for qi in q])

# Euler-Lagrange equations to obtain full dynamics M ddq + h = S^T tau
ddq = sp.symbols("ddx_b ddy_b ddth_b ddq1 ddq2")
ddq = sp.Matrix(ddq)

# Here we build E(q, dq, ddq) = 0 and identify h as terms independent of ddq
E = sp.Matrix([
    sp.diff(sp.diff(L, dq_i), 't') - sp.diff(L, q_i)
    for dq_i, q_i in zip(dq, q)
])
# In practice, replace 't' derivatives using chain rule and match coefficients
# of ddq to M_sym, and the remaining terms to h_sym.

# Contact Jacobian and its time derivative (used as J_c and Jdot_c dq)
Jc_sym = J_cf
Jc_dot_dq_sym = sp.Matrix([sp.diff(Jc_sym[i, j], q_k) * dq[k]
                           for i in range(Jc_sym.rows)
                           for j in range(Jc_sym.cols)
                           for k, q_k in enumerate(q)])
# Reshape into (2 x 5) matrix for Jdot_c dq
Jc_dot_dq_sym = sp.Matrix(2, 5, Jc_dot_dq_sym)

# Lambdify numeric functions (M, g, Jc, Jc_dot_dq)
M_fun = sp.lambdify((q, m_b, m1, m2, I_b, I1, I2, l1, l2, c1, c2), M_sym, "numpy")
g_fun = sp.lambdify((q, m_b, m1, m2, l1, l2, c1, c2, g), g_sym, "numpy")
Jc_fun = sp.lambdify((q, l1, l2), Jc_sym, "numpy")
Jc_dot_dq_fun = sp.lambdify((q, dq, l1, l2), Jc_dot_dq_sym, "numpy")

# --- Numeric constrained dynamics (stance phase) ---

def constrained_dynamics(q_val, dq_val, tau_val, params):
    """
    q_val: shape (5,)
    dq_val: shape (5,)
    tau_val: shape (2,)
    params: dict with masses, inertias, lengths, c offsets, gravity
    returns: ddq (5,), lambda_contact (2,)
    """
    m_bv, m1v, m2v = params["m_b"], params["m1"], params["m2"]
    I_bv, I1v, I2v = params["I_b"], params["I1"], params["I2"]
    l1v, l2v = params["l1"], params["l2"]
    c1v, c2v = params["c1"], params["c2"]
    gv = params["g"]

    q_np = np.asarray(q_val, dtype=float).reshape(5, 1)
    dq_np = np.asarray(dq_val, dtype=float).reshape(5, 1)
    tau_np = np.asarray(tau_val, dtype=float).reshape(2, 1)

    M = np.array(M_fun(q_np, m_bv, m1v, m2v, I_bv, I1v, I2v, l1v, l2v, c1v, c2v), dtype=float)
    g_vec = np.array(g_fun(q_np, m_bv, m1v, m2v, l1v, l2v, c1v, c2v, gv), dtype=float)

    # Placeholder: you would also compute Coriolis/Centrifugal from the full Lagrangian
    h = g_vec  # for slow motions near rest, Coriolis approx. negligible

    Jc = np.array(Jc_fun(q_np, l1v, l2v), dtype=float)
    Jc_dot_dq = np.array(Jc_dot_dq_fun(q_np, dq_np, l1v, l2v), dtype=float)

    # Selection matrix S (2 x 5)
    S = np.array([[0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 1]], dtype=float)

    # Build and solve the linear system
    Z = np.zeros((2, 2))
    A = np.block([[M, -Jc.T],
                  [Jc, Z]])
    rhs = np.vstack([S.T @ tau_np - h,
                     -Jc_dot_dq])

    sol = np.linalg.solve(A, rhs)
    ddq = sol[:5, :].reshape(5)
    lam = sol[5:, :].reshape(2)
    return ddq, lam
      
