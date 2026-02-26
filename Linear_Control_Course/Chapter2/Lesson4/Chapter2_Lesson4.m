syms t s J b omega0 real positive

omega = omega0 * exp(-b/J * t);          % omega(t)
Omega = laplace(omega, t, s);           % Omega(s)
lhs   = laplace(diff(omega, t), t, s);  % L{d omega/dt}
rhs   = s * Omega - subs(omega, t, 0);  % s*Omega(s) - omega(0+)

disp('L{d omega/dt} - (s*Omega(s) - omega(0+)) =');
disp(simplify(lhs - rhs))              % should be 0
