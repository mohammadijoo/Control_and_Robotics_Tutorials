% Chapter29_Lesson1.m
% Linear time-varying (LTV) state-space simulation:
%     x_dot = A(t)x + B(t)u(t)
%     y     = C(t)x + D(t)u(t)

clear; clc; close all;

tspan = [0, 20];
x0 = [1; 0];

opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);
[t, x] = ode45(@rhs_state, tspan, x0, opts);

y = zeros(size(t));
for k = 1:length(t)
    [~, ~, C, D] = ltv_matrices(t(k));
    u = input_signal(t(k));
    y(k) = C * x(k, :)' + D * u;
end

figure;
plot(t, x(:, 1), 'LineWidth', 1.5); hold on;
plot(t, x(:, 2), 'LineWidth', 1.5);
plot(t, y, '--', 'LineWidth', 1.5);
grid on;
xlabel('time');
ylabel('response');
title('Chapter 29 Lesson 1: LTV state-space response');
legend('x_1(t)', 'x_2(t)', 'y(t)');

% Optional Simulink guidance:
% Implement A(t), B(t), C(t), D(t) as MATLAB Function blocks, use an
% Integrator block for x_dot, and feed x and u into y = C(t)x + D(t)u.

function dx = rhs_state(t, x)
    [A, B, ~, ~] = ltv_matrices(t);
    u = input_signal(t);
    dx = A * x + B * u;
end

function [A, B, C, D] = ltv_matrices(t)
    omega = 1.0 + 0.30 * sin(0.70 * t);
    damping = 0.15 + 0.05 * cos(0.50 * t);

    A = [0, 1;
        -(omega^2), -2.0 * damping * omega];

    B = [0;
        1.0 + 0.20 * sin(0.30 * t)];

    C = [1.0 + 0.10 * sin(0.40 * t), 0];
    D = 0;
end

function u = input_signal(t)
    u = sin(1.2 * t);
end
