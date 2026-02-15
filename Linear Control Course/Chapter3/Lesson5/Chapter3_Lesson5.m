syms theta omega u m L b g real

x = [theta; omega];

f1 = omega;
f2 = -(b/(m*L^2))*omega - (g/L)*sin(theta) + u/(m*L^2);
f  = [f1; f2];

A = jacobian(f, x);
B = jacobian(f, u);

% Operating point (pendulum hanging down)
theta_op = 0;
omega_op = 0;
u_op     = 0;

A_op = subs(A, {theta, omega, u}, {theta_op, omega_op, u_op});
B_op = subs(B, {theta, omega, u}, {theta_op, omega_op, u_op});

% Example numerical parameters
A_num = double(subs(A_op, {m, L, b, g}, {1, 1, 0.1, 9.81}))
B_num = double(subs(B_op, {m, L}, {1, 1}))
