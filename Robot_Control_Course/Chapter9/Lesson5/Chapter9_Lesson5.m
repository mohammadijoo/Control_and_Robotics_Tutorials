
% Parameters
m = 1.0;
ell = 1.0;
I = m * ell^2;
b = 0.1;
g = 9.81;
dt = 0.01;

q_star = 0.0;
dq_star = 0.0;
u_star = m * g * ell * sin(q_star);
x_star = [q_star; dq_star];

% Linearization
A_c = [0, 1;
       -m * g * ell / I * cos(q_star), -b / I];
B_c = [0;
       1 / I];

A_d = eye(2) + dt * A_c;
B_d = dt * B_c;

Q = diag([10, 1]);
Qf = diag([50, 5]);
R = 0.1;

N = 500;
P = cell(N + 1, 1);
K = cell(N, 1);
P{N + 1} = Qf;

for k = N:-1:1
    S = R + B_d' * P{k + 1} * B_d;
    K{k} = -S \ (B_d' * P{k + 1} * A_d);
    Acl = A_d + B_d * K{k};
    P{k} = Q + Acl' * P{k + 1} * Acl;
end

% Simulation
steps = 500;
x = [0.5; 0.0];
xs = zeros(2, steps + 1);
us = zeros(1, steps);
xs(:, 1) = x;

for k = 1:steps
    dx = x - x_star;
    Kk = K{min(k, N)};
    u = u_star + Kk * dx;
    % Forward Euler integration of nonlinear dynamics
    q = x(1);
    dq = x(2);
    ddq = (u - b * dq - m * g * ell * sin(q)) / I;
    x = x + dt * [dq; ddq];
    xs(:, k + 1) = x;
    us(k) = u;
end

% Plot results
t = dt * (0:steps);
figure; subplot(2,1,1); plot(t, xs(1,:), t, xs(2,:));
legend('q', 'dq'); grid on;
subplot(2,1,2); plot(t(1:end-1), us); grid on; ylabel('tau');
