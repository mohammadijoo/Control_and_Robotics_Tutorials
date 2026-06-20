import sympy as sp

# Symbols
theta, omega, u = sp.symbols('theta omega u')
m, L, b, g = sp.symbols('m L b g', positive=True)

# State and dynamics
x = sp.Matrix([theta, omega])

f1 = omega
f2 = -(b/(m*L**2))*omega - (g/L)*sp.sin(theta) + u/(m*L**2)
f = sp.Matrix([f1, f2])

# Jacobians: A = df/dx, B = df/du
A = f.jacobian(x)
B = f.jacobian(sp.Matrix([u]))

print("A(theta,omega,u) =")
sp.pprint(A)
print("\nB(theta,omega,u) =")
sp.pprint(B)

# Operating point (pendulum hanging down with zero torque)
theta_op = 0
omega_op = 0
u_op = 0

subs_dict = {theta: theta_op, omega: omega_op, u: u_op}
A_op = sp.simplify(A.subs(subs_dict))
B_op = sp.simplify(B.subs(subs_dict))

print("\nLinearized A matrix at (0,0,0) =")
sp.pprint(A_op)
print("\nLinearized B matrix at (0,0,0) =")
sp.pprint(B_op)

# Example numerical parameters (typical small pendulum)
A_num = A_op.subs({m: 1.0, L: 1.0, b: 0.1, g: 9.81})
B_num = B_op.subs({m: 1.0, L: 1.0})

print("\nA_num =")
sp.pprint(A_num)
print("\nB_num =")
sp.pprint(B_num)
