% Chapter7_Lesson4.m
% Modern Control — Chapter 7, Lesson 4
% Output Response Using x(t) and y(t) = Cx(t) + Du(t)
%
% Demonstrates:
% 1) Closed-form y(t) for constant u(t)=u0 when A is invertible
% 2) Simulation using lsim (state-space object)
% 3) Optional Simulink model construction (commented section)

clear; clc;

% Example: 2-state SISO system
A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = 0.25;   % direct feedthrough to show output jump for step changes

x0 = [1; 0];
u0 = 1;

t = linspace(0, 6, 601).';

% Closed-form for constant input (requires invertible A):
% x(t) = expm(A t) x0 + A^{-1}(expm(A t) - I) B u0
% y(t) = C x(t) + D u0
I = eye(size(A));
if rcond(A) < 1e-12
    error('A is not invertible; constant-input closed form not applicable.');
end
Ainv = inv(A);

y_closed = zeros(length(t), 1);
x_closed = zeros(length(t), 2);

for k = 1:length(t)
    Phi = expm(A*t(k));
    xk = Phi*x0 + (Ainv*(Phi - I)*B)*u0;
    yk = C*xk + D*u0;
    x_closed(k,:) = xk.';
    y_closed(k) = yk;
end

disp('Closed-form y(t) at a few times:');
for ti = [0 0.5 1 2 6]
    [~,k] = min(abs(t-ti));
    fprintf('t=%.2f, y=%.8f\n', t(k), y_closed(k));
end

% Simulation using lsim
sys = ss(A,B,C,D);
u = ones(size(t)); % step
[y_lsim, t_out, x_lsim] = lsim(sys, u, t, x0);

disp('lsim y(t) at a few times:');
for ti = [0 0.5 1 2 6]
    [~,k] = min(abs(t_out-ti));
    fprintf('t=%.2f, y=%.8f\n', t_out(k), y_lsim(k));
end

% Plot
figure; plot(t, y_closed, t_out, y_lsim, '--'); grid on;
xlabel('t (s)'); ylabel('y(t)');
legend('Closed-form', 'lsim');

% ------------------------------------------------------------
% Optional: Programmatically build a Simulink model for y(t)
% ------------------------------------------------------------
%{
model = 'Chapter7_Lesson4_Simulink';
new_system(model); open_system(model);

add_block('simulink/Sources/Step', [model '/Step']);
add_block('simulink/Continuous/State-Space', [model '/StateSpace']);
add_block('simulink/Sinks/Scope', [model '/Scope']);

set_param([model '/StateSpace'], 'A', mat2str(A), 'B', mat2str(B), 'C', mat2str(C), 'D', mat2str(D));

add_line(model, 'Step/1', 'StateSpace/1');
add_line(model, 'StateSpace/1', 'Scope/1');

set_param(model, 'StopTime', '6');
save_system(model);

sim(model);
%}
