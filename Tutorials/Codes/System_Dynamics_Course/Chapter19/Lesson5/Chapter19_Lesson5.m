% Chapter19_Lesson5.m
% Spatio-Temporal Dynamics: method of lines for 1D heat equation + LQR feedback
%
% PDE: x_t = alpha x_xx + b(x) u(t),  x(0,t)=x(L,t)=0.
% Discretize to x_dot = A x + B u, then use LQR: u = -K x.

clear; clc;

alpha = 0.12;
L = 1.0;
N = 60;                 % interior nodes
dx = L/(N+1);

% Laplacian (Dirichlet)
e = ones(N,1);
D2 = spdiags([e -2*e e], [-1 0 1], N, N) / (dx^2);
A = alpha * full(D2);

xgrid = dx*(1:N)';

% Actuator shape (Gaussian)
x0 = 0.25; sigma = 0.07;
b = exp(-0.5*((xgrid-x0)/sigma).^2);
b = b / norm(b);
B = b; % (N x 1)

Q = eye(N);
R = 2e-3;

% Continuous-time LQR
% Requires Control System Toolbox: [K,~,~] = lqr(A,B,Q,R);
[K,~,~] = lqr(A,B,Q,R);

% Initial condition: smooth bump
x_init = exp(-80*(xgrid-0.75).^2);

% Simulate with forward Euler (for teaching purposes)
dt = 2e-3;
T  = 4.0;
steps = floor(T/dt);

x = x_init;
E = zeros(steps+1,1);
E(1) = 0.5*(x'*x);

for k = 1:steps
    u = -K*x;
    xdot = A*x + B*u;
    x = x + dt*xdot;
    E(k+1) = 0.5*(x'*x);
end

fprintf('Energy E(0)   = %.6e\n', E(1));
fprintf('Energy E(end) = %.6e\n', E(end));
fprintf('Decay factor  = %.6e\n', E(end)/E(1));

% Optional plot
figure; plot(linspace(0,T,steps+1), E);
xlabel('t'); ylabel('E(t)=0.5*||x||^2'); grid on;
title('Closed-loop energy decay (finite-dimensional LQR on PDE discretization)');
