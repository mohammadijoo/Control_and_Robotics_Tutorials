syms t s tau U0 x(t)

% Define the ODE: tau * dx/dt + x(t) = U0, x(0) = 0
ode = tau * diff(x, t) + x == U0;

% Laplace transform both sides
X = laplace(x, t, s);
U = laplace(U0, t, s);   % = U0/s

% Use derivative property manually:
% L{diff(x,t)} = s * X - x(0)
x0 = sym('x0');
laplaceEq = tau * (s * X - x0) + X == U;

% Apply initial condition x(0) = 0
laplaceEq0 = subs(laplaceEq, x0, 0);
X_s = solve(laplaceEq0, X);           % solve algebraic equation for X(s)
X_s_simplified = simplify(X_s)

% Inverse Laplace to get x(t)
x_t = ilaplace(X_s_simplified, s, t)
pretty(x_t)
