% Chapter29_Lesson5.m
%
% Slowly time-varying mass-spring-damper example for MATLAB/Simulink-oriented study.
%
% Required MATLAB functions:
%   ode45, eig, plot, writematrix
%
% Optional Simulink mapping:
%   x1_dot = x2
%   x2_dot = -(k(t)/m(t))*x1 - (c(t)/m(t))*x2 + u(t)/m(t)
% Use two Integrator blocks, MATLAB Function blocks for m(t), c(t), k(t), and u(t),
% and Sum/Gain/Product blocks from the Simulink Continuous and Math Operations libraries.

clear; clc; close all;

epsilon = 0.03;

mass = @(t) 1.0 + 0.12*sin(epsilon*t);
damping = @(t) 0.22 + 0.04*cos(epsilon*t);
stiffness = @(t) 2.0 + 0.30*sin(0.5*epsilon*t);
inputForce = @(t) 0.2*sin(0.7*t);

A = @(t) [0, 1; -stiffness(t)/mass(t), -damping(t)/mass(t)];
B = @(t) [0; 1/mass(t)];

rhs = @(t, x) A(t)*x + B(t)*inputForce(t);

tspan = [0, 160];
x0 = [1; 0];

options = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);
[t, x] = ode45(rhs, tspan, x0, options);

alpha = zeros(size(t));
for i = 1:length(t)
    lambda = eig(A(t(i)));
    alpha(i) = -max(real(lambda));
end

output = [t, x(:,1), x(:,2), alpha];
writematrix(output, 'Chapter29_Lesson5_matlab_output.csv');

fprintf('Final state: x1 = %.8f, x2 = %.8f\n', x(end,1), x(end,2));
fprintf('Minimum frozen-time decay margin alpha = %.8f\n', min(alpha));

figure;
plot(t, x(:,1), 'LineWidth', 1.4);
grid on;
xlabel('time');
ylabel('displacement');
title('Slowly Time-Varying Mass-Spring-Damper System');

figure;
plot(t, alpha, 'LineWidth', 1.4);
grid on;
xlabel('time');
ylabel('frozen-time decay margin');
title('Frozen-Time Stability Margin');
