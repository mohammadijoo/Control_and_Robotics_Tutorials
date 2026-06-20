import sympy as sp
import numpy as np

# Symbolic variables
q1, q2, l1, l2, xd, yd = sp.symbols('q1 q2 l1 l2 xd yd', real=True)

# Forward kinematics for planar 2R
x = l1*sp.cos(q1) + l2*sp.cos(q1 + q2)
y = l1*sp.sin(q1) + l2*sp.sin(q1 + q2)

q = sp.Matrix([q1, q2])
f = sp.Matrix([x, y])

# Jacobian J(q)
J = f.jacobian(q)

# Task-space error and cost
e = f - sp.Matrix([xd, yd])
phi = sp.Rational(1, 2) * (e.dot(e))

# Gradient and Hessian in joint space
grad_phi = sp.Matrix([sp.diff(phi, q1), sp.diff(phi, q2)])
H_phi = sp.hessian(phi, q)

print("J(q) =")
sp.pprint(J)
print("grad_phi(q) =")
sp.pprint(grad_phi)
print("H_phi(q) =")
sp.pprint(H_phi)

# Numeric evaluation
subs_vals = {
    l1: 1.0, l2: 0.8,
    xd: 1.2, yd: 0.3,
    q1: 0.5, q2: -0.3
}
J_num = np.array(J.evalf(subs=subs_vals), dtype=float)
grad_num = np.array(grad_phi.evalf(subs=subs_vals), dtype=float).reshape(-1)
H_num = np.array(H_phi.evalf(subs=subs_vals), dtype=float)

print("J_num =", J_num)
print("grad_num =", grad_num)
print("H_num =", H_num)
      
