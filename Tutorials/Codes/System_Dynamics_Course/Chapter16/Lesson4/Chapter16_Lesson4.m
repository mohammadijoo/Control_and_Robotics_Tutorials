% Chapter16_Lesson4.m
% Continuous–Discrete Conversions: Zero-Order Hold, Exact Discretization
% MATLAB / Simulink-oriented implementation

clear; clc; close all;

% Mass-spring-damper example
m = 1.0; c = 0.6; k = 4.0; b = 1.0;
A = [0 1; -k/m -c/m];
B = [0; b/m];
C = [1 0];
D = 0;

Ts = 0.1;

% Exact ZOH discretization via augmented matrix exponential
n = size(A,1);
m_in = size(B,2);
M = [A B; zeros(m_in,n) zeros(m_in,m_in)];
Md = expm(M*Ts);
Ad_exact = Md(1:n,1:n);
Bd_exact = Md(1:n,n+1:n+m_in);

% Closed-form when A is nonsingular: Bd = A^{-1}(Ad - I)B
if rank(A) == n
    Bd_closed = A \ ((Ad_exact - eye(n))*B);
else
    Bd_closed = NaN(size(B));
end

% Toolbox method (Control System Toolbox): c2d with ZOH
sysc = ss(A,B,C,D);
sysd = c2d(sysc, Ts, 'zoh');

Ad_c2d = sysd.A;
Bd_c2d = sysd.B;

disp('Ad_exact ='); disp(Ad_exact);
disp('Bd_exact ='); disp(Bd_exact);
disp('Ad_c2d =');  disp(Ad_c2d);
disp('Bd_c2d =');  disp(Bd_c2d);

fprintf('||Ad_exact - Ad_c2d||_F = %.3e\n', norm(Ad_exact - Ad_c2d, 'fro'));
fprintf('||Bd_exact - Bd_c2d||_F = %.3e\n', norm(Bd_exact - Bd_c2d, 'fro'));

% Compare exact ZOH and forward Euler discretization
Ad_euler = eye(n) + Ts*A;
Bd_euler = Ts*B;

N = 120;
u = zeros(N,1);
u(6:end) = 1;  % unit step after k=5
t = (0:N-1)'*Ts;

x_exact = zeros(n,1);
x_euler = zeros(n,1);
y_exact = zeros(N,1);
y_euler = zeros(N,1);

for kstep = 1:N
    y_exact(kstep) = C*x_exact + D*u(kstep);
    y_euler(kstep) = C*x_euler + D*u(kstep);

    x_exact = Ad_exact*x_exact + Bd_exact*u(kstep);
    x_euler = Ad_euler*x_euler + Bd_euler*u(kstep);
end

figure;
plot(t, y_exact, 'LineWidth', 1.5); hold on;
plot(t, y_euler, '--', 'LineWidth', 1.3);
stairs(t, u, ':', 'LineWidth', 1.0);
grid on;
xlabel('Time [s]');
ylabel('Output / Input');
title('Exact ZOH vs Forward Euler Discretization');
legend('Exact ZOH','Forward Euler','u[k]', 'Location','best');

% Simulink note (manual workflow):
% 1) Build a continuous State-Space block with (A,B,C,D).
% 2) Add a Zero-Order Hold block with sample time Ts at the input.
% 3) Compare against a Discrete State-Space block using (Ad_exact,Bd_exact,C,D).
% 4) The sampled outputs should match at t = k*Ts (up to numerical tolerances).
