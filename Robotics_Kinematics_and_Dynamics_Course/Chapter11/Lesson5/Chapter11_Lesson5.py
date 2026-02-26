import sympy as sp
import numpy as np

# 1. Symbolic definitions
q1, q2 = sp.symbols('q1 q2', real=True)
dq1, dq2 = sp.symbols('dq1 dq2', real=True)
g, l1, l2, lc1, lc2, m1, m2, I1, I2 = sp.symbols(
    'g l1 l2 lc1 lc2 m1 m2 I1 I2', positive=True, real=True
)

q = sp.Matrix([q1, q2])
dq = sp.Matrix([dq1, dq2])

# 2. Kinetic energy T and potential energy V (from Section 2)
M11 = I1 + I2 + m1*lc1**2 + m2*(l1**2 + lc2**2 + 2*l1*lc2*sp.cos(q2))
M12 = I2 + m2*(lc2**2 + l1*lc2*sp.cos(q2))
M22 = I2 + m2*lc2**2

M = sp.Matrix([[M11, M12],
               [M12, M22]])

T = sp.Rational(1, 2) * dq.T * M * dq
T = sp.simplify(T[0])  # scalar

V = (m1*lc1 + m2*l1)*g*sp.cos(q1) + m2*lc2*g*sp.cos(q1 + q2)

L = T - V

# 3. Euler-Lagrange equations to get closed-form dynamics
tau = sp.symbols('tau1 tau2', real=True)

dLd_dq = [sp.diff(L, q_i) for q_i in q]
dLd_ddq = [sp.diff(L, dq_i) for dq_i in dq]

# Time derivatives d/dt(dL/ddq_i): treat q, dq as independent and replace dq, ddq
ddq1, ddq2 = sp.symbols('ddq1 ddq2', real=True)
ddq = sp.Matrix([ddq1, ddq2])

# Here we use the known structure T = 0.5 * dq.T * M(q) * dq
# so that M(q) is already identified; we directly build M, C, g:
g_vec = sp.Matrix([sp.diff(V, q1), sp.diff(V, q2)])

# Christoffel-based C(q, dq):
C = sp.zeros(2, 2)
for i in range(2):
    for j in range(2):
        Cij = 0
        for k in range(2):
            c_ijk = 0.5 * (
                sp.diff(M[i, j], q[k]) +
                sp.diff(M[i, k], q[j]) -
                sp.diff(M[j, k], q[i])
            )
            Cij += c_ijk * dq[k]
        C[i, j] = sp.simplify(Cij)

# 4. Lambdify numeric versions M(q), C(q, dq), g(q)
M_func = sp.lambdify((q1, q2, l1, l2, lc1, lc2, m1, m2, I1, I2),
                     M, 'numpy')
C_func = sp.lambdify((q1, q2, dq1, dq2, l1, l2, lc1, lc2, m1, m2, I1, I2),
                     C, 'numpy')
g_func = sp.lambdify((q1, q2, g, l1, l2, lc1, lc2, m1, m2),
                     g_vec, 'numpy')

# 5. Numeric finite-difference approximation for M(q) as a check
def T_numeric(q_vec, dq_vec, params):
    # params = (g, l1, l2, lc1, lc2, m1, m2, I1, I2)
    g_, l1_, l2_, lc1_, lc2_, m1_, m2_, I1_, I2_ = params
    q1_, q2_ = q_vec
    dq1_, dq2_ = dq_vec
    M_num = M_func(q1_, q2_, l1_, l2_, lc1_, lc2_, m1_, m2_, I1_, I2_)
    return 0.5 * dq_vec.T @ M_num @ dq_vec

def M_fd(q_vec, params, h=1e-6):
    n = len(q_vec)
    M_fd = np.zeros((n, n))
    dq0 = np.zeros(n)
    # use central differences on dq only, around dq0
    for i in range(n):
        for j in range(n):
            ei = np.zeros(n); ei[i] = 1.0
            ej = np.zeros(n); ej[j] = 1.0
            Tpp = T_numeric(q_vec, dq0 + h*ei + h*ej, params)
            Tpm = T_numeric(q_vec, dq0 + h*ei - h*ej, params)
            Tmp = T_numeric(q_vec, dq0 - h*ei + h*ej, params)
            Tmm = T_numeric(q_vec, dq0 - h*ei - h*ej, params)
            M_fd[i, j] = (Tpp - Tpm - Tmp + Tmm) / (4*h*h)
    return 0.5*(M_fd + M_fd.T)  # symmetrize

# 6. Random test
params_num = (9.81, 1.0, 1.0, 0.5, 0.5, 2.0, 1.0, 0.1, 0.1)
q_vec = np.array([0.3, -0.7])

M_sym = M_func(q_vec[0], q_vec[1], *params_num[1:])
M_fd_ = M_fd(q_vec, params_num)

print("M_sym =\n", M_sym)
print("M_fd  =\n", M_fd_)
print("diff norm:", np.linalg.norm(M_sym - M_fd_))
      
