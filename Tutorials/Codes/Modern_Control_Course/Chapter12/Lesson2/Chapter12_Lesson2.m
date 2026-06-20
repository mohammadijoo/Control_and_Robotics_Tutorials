% Chapter12_Lesson2.m
% Minimum-energy steering for continuous-time LTI systems.
% MATLAB implementation plus Simulink-oriented setup variables.

clear; clc;

A = [0 1;
     0 0];
B = [0;
     1];

T  = 2.0;
x0 = [0; 0];
xf = [1; 0];

% Compute Wc(T) from the differential Lyapunov equation:
% dW/dt = A W + W A' + B B', W(0) = 0.
n = size(A, 1);
gramian_ode = @(t, w) reshape(A*reshape(w, n, n) + reshape(w, n, n)*A' + B*B', [], 1);
[~, Wvec] = ode45(gramian_ode, [0 T], zeros(n*n, 1));
W = reshape(Wvec(end, :).', n, n);

z = xf - expm(A*T)*x0;
lambda = W \ z;

u_star = @(t) B' * expm(A'*(T - t)) * lambda;
E_min = z' * lambda;

fprintf('Wc(T) =\n');
disp(W);
fprintf('condition number of Wc(T) = %.6e\n', cond(W));
fprintf('minimum energy = %.12f\n', E_min);

% Simulate x_dot = A x + B u_star(t).
rhs = @(t, x) A*x + B*u_star(t);
[tgrid, xgrid] = ode45(rhs, linspace(0, T, 501), x0);

fprintf('terminal state reached =\n');
disp(xgrid(end, :).');
fprintf('target state =\n');
disp(xf);
fprintf('terminal error norm = %.6e\n', norm(xgrid(end, :).'-xf));

% Plot state and optimal input.
figure;
plot(tgrid, xgrid, 'LineWidth', 1.5);
grid on;
xlabel('time');
ylabel('states');
legend('x_1 position', 'x_2 velocity');

uvals = arrayfun(u_star, tgrid);
figure;
plot(tgrid, uvals, 'LineWidth', 1.5);
grid on;
xlabel('time');
ylabel('u^*(t)');

% Simulink implementation idea:
% 1. Use a State-Space block with matrices A, B, C = eye(2), D = zeros(2,1).
% 2. Use a MATLAB Function block to compute:
%       u = B' * expm(A'*(T - t)) * lambda
%    where T, A, B, and lambda are workspace variables.
% 3. Feed Clock into the MATLAB Function block and feed u into the State-Space block.
% 4. Use Scope blocks to compare x(T) with xf.
