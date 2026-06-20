import sympy as sp

# Symbols
t, s = sp.symbols('t s', real=True, positive=True)
J, b = sp.symbols('J b', positive=True)
omega0 = sp.symbols('omega0')

# Example joint speed signal (free response of first-order system)
omega_t = omega0 * sp.exp(-b/J * t)

# Laplace transform of omega(t)
Omega_s = sp.laplace_transform(omega_t, t, s)[0]

# Laplace of derivative d omega/dt
lhs = sp.laplace_transform(sp.diff(omega_t, t), t, s)[0]

# s*Omega(s) - omega(0+)
rhs = s * Omega_s - omega_t.subs(t, 0)

print("Omega(s) =", Omega_s)
print("L{d omega/dt} =", lhs)
print("s*Omega(s) - omega(0+) =", rhs)
print("Identity holds:", sp.simplify(lhs - rhs) == 0)
