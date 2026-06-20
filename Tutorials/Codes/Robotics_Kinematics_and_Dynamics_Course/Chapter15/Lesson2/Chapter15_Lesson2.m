function dx = circle_dynamics(t, x, m, l, g)
% x = [q; qdot] = [x; y; xdot; ydot]

q    = x(1:2);
qdot = x(3:4);

M = m * eye(2);
J = [2*q(1), 2*q(2)];

% Jdot * qdot for time-invariant constraint
Jdot_qdot = 2*(qdot(1)^2 + qdot(2)^2);
gamma = -Jdot_qdot;

% Unconstrained generalized forces (gravity)
f = [0; -m*g];
tau = [0; 0];

A = [M, -J'; J, 0];
rhs = [tau + f; gamma];

sol = A \ rhs;
qddot = sol(1:2);  % constrained acceleration

dx = [qdot; qddot];
end

% Example of numerical integration
m = 1.0; l = 1.0; g = 9.81;
x0 = [1.0; 0.0; 0.0; 1.0];  % initial [x; y; xdot; ydot]
[t, x] = ode45(@(t, x) circle_dynamics(t, x, m, l, g), [0 10], x0);
plot(t, x(:,1), t, x(:,2));
legend('x', 'y');
axis equal;
      
