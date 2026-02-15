import sympy as sp

# Generalized coordinates and velocities for a 2R planar arm
q1, q2 = sp.symbols("q1 q2")
dq1, dq2 = sp.symbols("dq1 dq2")
q = sp.Matrix([q1, q2])
dq = sp.Matrix([dq1, dq2])

# Simple link parameters (masses and lengths)
m1, m2 = sp.symbols("m1 m2", positive=True)
l1, l2 = sp.symbols("l1 l2", positive=True)
g = sp.symbols("g", real=True)

# Kinetic energy (planar 2R, COM at l_i/2, scalar inertias I1, I2)
I1, I2 = sp.symbols("I1 I2", positive=True)

# Position of COMs in the plane
x1 = (l1 / 2) * sp.cos(q1)
y1 = (l1 / 2) * sp.sin(q1)

x2 = l1 * sp.cos(q1) + (l2 / 2) * sp.cos(q1 + q2)
y2 = l1 * sp.sin(q1) + (l2 / 2) * sp.sin(q1 + q2)

# Velocities of COMs via Jacobians
J1 = sp.Matrix([[sp.diff(x1, q1), sp.diff(x1, q2)],
                [sp.diff(y1, q1), sp.diff(y1, q2)]])
J2 = sp.Matrix([[sp.diff(x2, q1), sp.diff(x2, q2)],
                [sp.diff(y2, q1), sp.diff(y2, q2)]])

v1 = J1 * dq
v2 = J2 * dq

T_trans = sp.Rational(1, 2) * m1 * (v1.dot(v1)) \
        + sp.Rational(1, 2) * m2 * (v2.dot(v2))

# Angular velocities about z-axis
w1 = dq1
w2 = dq1 + dq2
T_rot = sp.Rational(1, 2) * I1 * w1**2 + sp.Rational(1, 2) * I2 * w2**2

T = sp.simplify(T_trans + T_rot)

# Potential energy (gravity in -y direction)
V = m1 * g * y1 + m2 * g * y2

L = T - V

# Extract inertia matrix M(q) from T(q, dq) = 0.5 dq^T M dq
M = sp.hessian(T, (dq1, dq2))  # Hessian of T w.r.t. velocities
M = sp.simplify(M)

print("Inertia matrix M(q):")
sp.pprint(M)

# Compute Christoffel symbols from the metric M(q)
q_symbols = (q1, q2)
M_inv = sp.simplify(M.inv())
Gamma = [[[0 for _ in range(2)] for _ in range(2)] for _ in range(2)]

for k in range(2):
    for i in range(2):
        for j in range(2):
            term = 0
            for ell in range(2):
                term += M_inv[k, ell] * (
                    sp.diff(M[j, ell], q_symbols[i]) +
                    sp.diff(M[i, ell], q_symbols[j]) -
                    sp.diff(M[i, j], q_symbols[ell])
                )
            Gamma[k][i][j] = sp.simplify(sp.Rational(1, 2) * term)

print("Christoffel symbols Gamma^k_ij(q):")
for k in range(2):
    for i in range(2):
        for j in range(2):
            print(f"Gamma[{k+1}][{i+1}][{j+1}] = {Gamma[k][i][j]}")

# Euler-Lagrange equations (Lagrange-d'Alembert with zero non-conservative forces)
ddq1, ddq2 = sp.symbols("ddq1 ddq2")
ddq = sp.Matrix([ddq1, ddq2])

EL = []
for k, qk in enumerate(q_symbols):
    dL_ddqk = sp.diff(L, (dq1, dq2)[k])
    d_dt_dL_ddqk = sp.diff(dL_ddqk, q1) * dq1 + sp.diff(dL_ddqk, q2) * dq2 \
                   + sp.diff(dL_ddqk, dq1) * ddq1 + sp.diff(dL_ddqk, dq2) * ddq2
    dL_dqk = sp.diff(L, qk)
    EL.append(sp.simplify(d_dt_dL_ddqk - dL_dqk))

print("Euler-Lagrange equations (symbolic):")
for k in range(2):
    print(f"E_{k+1}(q, dq, ddq) = {EL[k]}")
      
