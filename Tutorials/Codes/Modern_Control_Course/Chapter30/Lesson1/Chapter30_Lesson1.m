% Chapter30_Lesson1.m
% Descriptor (singular) system example: E xdot = A x + B u.
% This script computes generalized eigenvalues, performs an index-1 reduction,
% simulates the reduced ODE, and reconstructs the algebraic state.

clear; clc; close all;

E = diag([1 1 0]);
A = [ 0  1  0;
     -2 -3  1;
      1  0  1];
B = [0; 1; -1];

lambda = eig(A, E);
disp('Generalized eigenvalues of (A,E):');
disp(lambda);

A11 = A(1:2, 1:2);
A12 = A(1:2, 3);
A21 = A(3, 1:2);
A22 = A(3, 3);
B1 = B(1:2);
B2 = B(3);

% Algebraic equation: 0 = A21*xd + A22*xa + B2*u.
% Since A22 is nonsingular, xa = -A22^(-1)*A21*xd - A22^(-1)*B2*u.
Ar = A11 - A12 * (A22 \ A21);
Br = B1 - A12 * (A22 \ B2);

disp('Reduced matrix Ar:');
disp(Ar);
disp('Reduced input matrix Br:');
disp(Br);
disp('Reduced eigenvalues:');
disp(eig(Ar));

u = @(t) double(t >= 1.0);
rhs = @(t, xd) Ar*xd + Br*u(t);
reconstruct_xa = @(xd, input_value) -A22 \ (A21*xd + B2*input_value);

xd0 = [0.2; 0.0];
xa0 = reconstruct_xa(xd0, u(0));
disp('Consistent initial condition [xd; xa]:');
disp([xd0; xa0]);

[t, xd] = ode45(rhs, [0 8], xd0);
x3 = zeros(length(t), 1);
residual = zeros(length(t), 1);
for k = 1:length(t)
    x3(k) = reconstruct_xa(xd(k, :).', u(t(k)));
    residual(k) = A21*xd(k, :).'+ A22*x3(k) + B2*u(t(k));
end
fprintf('Maximum algebraic constraint residual: %.3e\n', max(abs(residual)));

figure;
plot(t, xd(:, 1), 'DisplayName', 'x1 differential'); hold on;
plot(t, xd(:, 2), 'DisplayName', 'x2 differential');
plot(t, x3, 'DisplayName', 'x3 algebraic');
grid on;
xlabel('time');
ylabel('state');
title('Descriptor system reduced to an index-1 ODE');
legend('Location', 'best');

% Simulink note:
% For a block diagram implementation, represent the first two equations by
% Integrator blocks and implement 0 = x1 + x3 - u with an Algebraic Constraint
% block or by explicitly computing x3 = -x1 + u for this index-1 example.
