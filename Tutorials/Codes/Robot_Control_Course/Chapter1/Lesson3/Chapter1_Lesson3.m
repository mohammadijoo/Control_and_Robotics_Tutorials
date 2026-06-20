
syms q qdot tau m g l I real

x = [q; qdot];
u = tau;

% Nonlinear dynamics
f1 = qdot;
f2 = -(m*g*l/I) * sin(q) + (1/I) * tau;
f  = [f1; f2];

% Jacobians
A = jacobian(f, x);
B = jacobian(f, u);

disp('A(q,qdot) =');
disp(A);
disp('B(q,qdot) =');
disp(B);

% Evaluate at equilibrium q* = 0, qdot* = 0, tau* = 0
A_eq = subs(A, {q, qdot, tau}, {0, 0, 0});
B_eq = subs(B, {q, qdot, tau}, {0, 0, 0});

disp('A at equilibrium:');
disp(A_eq);
disp('B at equilibrium:');
disp(B_eq);

% Substitute numeric parameters
params = {m, g, l, I};
vals   = {1.0, 9.81, 1.0, 1.0};
A_num  = double(subs(A_eq, params, vals));
B_num  = double(subs(B_eq, params, vals));

disp('Numeric A:');
disp(A_num);
disp('Numeric B:');
disp(B_num);
