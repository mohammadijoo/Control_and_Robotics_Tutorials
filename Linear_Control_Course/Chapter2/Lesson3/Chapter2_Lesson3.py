import sympy as sp

# Symbols
t, s = sp.symbols('t s', real=True, positive=True)
tau, U0 = sp.symbols('tau U0', positive=True)

# Time-domain variables
x = sp.Function('x')
u = U0  # Step input of magnitude U0 for t >= 0

# Define the ODE: tau * dx/dt + x(t) = u(t), with x(0) = 0
ode = sp.Eq(tau * sp.diff(x(t), t) + x(t), u)

# Take Laplace transform of both sides (unilateral)
X = sp.Function('X')
X_s = sp.Function('X')(s)  # placeholder

# Sympy provides laplace_transform directly
X_s = sp.laplace_transform(x(t), t, s, noconds=True)
U_s = sp.laplace_transform(u, t, s, noconds=True)  # = U0 / s

# Express the transformed ODE using the derivative property manually:
# L{dx/dt} = s * X(s) - x(0)
x0 = sp.Symbol('x0')
laplace_ode = sp.Eq(tau * (s * X_s - x0) + X_s, U_s)

# Assume x(0) = 0 and solve for X(s)
laplace_ode_zero_ic = laplace_ode.subs(x0, 0)
X_s_solution = sp.solve(laplace_ode_zero_ic, X_s)[0]
print("X(s) =", sp.simplify(X_s_solution))

# Inverse Laplace to get x(t)
x_t = sp.inverse_laplace_transform(X_s_solution, s, t)
print("x(t) =", sp.simplify(x_t))
