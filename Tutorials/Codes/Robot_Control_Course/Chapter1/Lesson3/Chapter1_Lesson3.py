
import sympy as sp

# Symbolic variables
q, qdot, tau = sp.symbols("q qdot tau")
m, g, l, I = sp.symbols("m g l I", positive=True)

# State and input vectors
x = sp.Matrix([q, qdot])
u = sp.Matrix([tau])

# Nonlinear dynamics: xdot = f(x,u)
f1 = qdot
f2 = -(m*g*l/I) * sp.sin(q) + (1/I) * tau
f = sp.Matrix([f1, f2])

# Jacobians
A = f.jacobian(x)
B = f.jacobian(u)

print("A(q,qdot) =")
sp.pprint(A)
print("B(q,qdot) =")
sp.pprint(B)

# Evaluate at downward equilibrium: q* = 0, qdot* = 0, tau* = 0
eq_point = {q: 0.0, qdot: 0.0, tau: 0.0}
A_eq = A.subs(eq_point)
B_eq = B.subs(eq_point)

print("A at equilibrium:")
sp.pprint(A_eq)
print("B at equilibrium:")
sp.pprint(B_eq)

# Numeric evaluation for specific parameters
params = {m: 1.0, g: 9.81, l: 1.0, I: 1.0}
A_num = A_eq.subs(params).evalf()
B_num = B_eq.subs(params).evalf()

print("Numeric A:")
sp.pprint(A_num)
print("Numeric B:")
sp.pprint(B_num)
