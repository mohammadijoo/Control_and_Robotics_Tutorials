import sympy as sp

# General n-DOF symbolic construction
n = 2
q1, q2 = sp.symbols('q1 q2')
dq1, dq2 = sp.symbols('dq1 dq2')
a1, a2, a3 = sp.symbols('a1 a2 a3')

q = sp.Matrix([q1, q2])
dq = sp.Matrix([dq1, dq2])

# Inertia matrix for 2R example
M = sp.Matrix([
    [a1 + 2*a2*sp.cos(q2), a3 + a2*sp.cos(q2)],
    [a3 + a2*sp.cos(q2),   a3]
])

# Christoffel symbols c[i][j][k]
c = [[[0 for k in range(n)] for j in range(n)] for i in range(n)]

for i in range(n):
    for j in range(n):
        for k in range(n):
            c[i][j][k] = sp.Rational(1, 2) * (
                sp.diff(M[i, j], q[k]) +
                sp.diff(M[i, k], q[j]) -
                sp.diff(M[j, k], q[i])
            )

# Coriolis matrix C(q, dq) with convention C_ij = sum_k c_ijk dq_k
C = sp.Matrix.zeros(n, n)
for i in range(n):
    for j in range(n):
        C[i, j] = sum(c[i][j][k] * dq[k] for k in range(n))

print("M(q) =")
sp.pprint(M)
print("\nC(q, dq) =")
sp.pprint(C)

# Optional: generate fast numerical functions (e.g., for real-time control)
M_fun = sp.lambdify((q1, q2, a1, a2, a3), M, "numpy")
C_fun = sp.lambdify((q1, q2, dq1, dq2, a2), C, "numpy")
      
