% Chapter16_Lesson5.m
% Discrete-time stability and time response (MATLAB / Simulink-oriented)
% Requires Control System Toolbox for tf/step; dlyap is in Control System Toolbox.

clear; clc;

Ts = 0.1;  % sampling period [s]

% A stable 2nd-order discrete transfer function with DC gain = 1
num = [0.0676 0.0604];
den = [1 -1.5770 0.6724];
Gz = tf(num, den, Ts);

disp('Discrete transfer function G(z):');
Gz

p = pole(Gz);
fprintf('Poles of G(z):\n');
disp(p.');
fprintf('Magnitudes of poles:\n');
disp(abs(p).');

if all(abs(p) < 1)
    fprintf('System is Schur stable (all poles inside unit circle).\n');
else
    fprintf('System is NOT Schur stable.\n');
end

% State-space form and discrete Lyapunov equation
sys_ss = ss(Gz);
A = sys_ss.A; B = sys_ss.B; C = sys_ss.C; D = sys_ss.D;
Q = eye(size(A));
P = dlyap(A', Q);  % solves A' P A - P + Q = 0
disp('Lyapunov matrix P from A'' P A - P = -Q:');
disp(P);

% Step response and basic metrics
N = 100;
k = 0:N-1;
t = k * Ts;
[y, t] = step(Gz, t);

figure;
stairs(t, y, 'LineWidth', 1.3); grid on;
xlabel('Time (s)'); ylabel('y[k]');
title('Unit-Step Response of Discrete-Time System');

info = stepinfo(y, t, 1.0, 'SettlingTimeThreshold', 0.02);
disp('Step response metrics:');
disp(info);

% Natural response (zero input, nonzero initial condition)
x0 = [1; -0.4];
u = zeros(N,1);
[y_nat, t_nat, x_nat] = lsim(sys_ss, u, t, x0);

figure;
stairs(t_nat, y_nat, 'LineWidth', 1.3); grid on;
xlabel('Time (s)'); ylabel('y_n[k]');
title('Natural Response (u=0, x_0=[1;-0.4])');

% Optional Simulink note:
% You can build an equivalent model with:
%   Step -> Discrete Transfer Fcn -> Scope
% and set numerator/denominator to the vectors above with sample time Ts.
