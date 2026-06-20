% Chapter9_Lesson4.m
% Internal stability versus external input-output behavior.
%
% Requires only base MATLAB for the simulations below.
% Optional Control System Toolbox commands are included when available.

clear; clc;

A = [-1 0; 0 1];
B = [1; 0];
C = [1 0];
D = 0;

fprintf('Eigenvalues of A:\n');
disp(eig(A));
fprintf('A has an unstable internal mode at +1.\n');
fprintf('The input-output transfer function is G(s)=1/(s+1), externally stable.\n');

% Optional Control System Toolbox verification
if exist('ss', 'file') == 2 && exist('tf', 'file') == 2
    sys = ss(A, B, C, D);
    fprintf('\nState-space model converted to transfer function:\n');
    disp(tf(sys));
end

% Case 1: zero input, hidden unstable initial condition
x0_hidden = [0; 1];
ode_zero = @(t, x) A*x + B*0;
[t1, x1] = ode45(ode_zero, [0 5], x0_hidden);
y1 = (C*x1.').';

fprintf('\nCase 1: u(t)=0, x(0)=[0;1]\n');
fprintf('  final state x(T) = [%g, %g]\n', x1(end,1), x1(end,2));
fprintf('  final output y(T) = %g\n', y1(end));
fprintf('  max |y(t)| = %g\n', max(abs(y1)));
fprintf('  max ||x(t)|| = %g\n', max(vecnorm(x1, 2, 2)));

% Case 2: unit step input, zero initial condition
x0_zero = [0; 0];
ode_step = @(t, x) A*x + B*1;
[t2, x2] = ode45(ode_step, [0 5], x0_zero);
y2 = (C*x2.').';

fprintf('\nCase 2: u(t)=1, x(0)=[0;0]\n');
fprintf('  final state x(T) = [%g, %g]\n', x2(end,1), x2(end,2));
fprintf('  final output y(T) = %g\n', y2(end));
fprintf('  expected y(T) approximately = %g\n', 1 - exp(-t2(end)));
fprintf('  max |y(t)| = %g\n', max(abs(y2)));

figure;
plot(t1, x1(:,2), 'LineWidth', 1.5); grid on;
xlabel('time'); ylabel('hidden state x_2(t)');
title('Internal instability: hidden state grows under zero input');

figure;
plot(t2, y2, 'LineWidth', 1.5); grid on;
xlabel('time'); ylabel('output y(t)');
title('Externally stable step response when hidden initial state is zero');
