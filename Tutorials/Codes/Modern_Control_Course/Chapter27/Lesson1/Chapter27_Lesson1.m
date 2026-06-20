% Chapter27_Lesson1.m
% Formulating Tracking Problems in State-Space Form
%
% This script formulates a continuous-time tracking problem
%
%   x_dot = A*x + B*u + E*d
%   y     = C*x + D*u + F*d
%   e     = y - r
%
% It computes constant-reference equilibrium equations, checks residual
% feasibility, and simulates a stabilizing deviation-feedback law.
%
% Toolboxes:
%   - The numerical simulation works in base MATLAB.
%   - The optional Simulink section requires Simulink.

clear; clc; close all;

A = [0 1; -2 -0.8];
B = [0; 1];
C = [1 0];
D = 0;
E = [0; 1];
F = 0;

r = 1.0;       % constant reference
d = 0.2;       % constant disturbance

% Steady-state tracking equations:
%   A*x_bar + B*u_bar + E*d = 0
%   C*x_bar + D*u_bar + F*d = r
M = [A B; C D];
rhs = [-E*d; r - F*d];

theta = pinv(M) * rhs;
x_bar = theta(1:2);
u_bar = theta(3);
residual = M * theta - rhs;

disp('Stacked equilibrium matrix [A B; C D]:');
disp(M);
disp('Computed x_bar:');
disp(x_bar);
disp('Computed u_bar:');
disp(u_bar);
fprintf('Equilibrium residual norm = %.8e\n', norm(residual));

% The gain K is not the topic of this lesson; it only stabilizes the
% formulated tracking equilibrium for demonstration.
K = [4.0 2.2];

closed_loop = @(t, x) A*x + B*(u_bar - K*(x - x_bar)) + E*d;

tspan = [0 8];
x0 = [0; 0];
[t, x] = ode45(closed_loop, tspan, x0);

y = (C*x')';
e = y - r;

fprintf('Final y(T) = %.8f\n', y(end));
fprintf('Final e(T) = %.8e\n', e(end));

figure;
plot(t, y, 'LineWidth', 1.5); hold on;
plot(t, r*ones(size(t)), '--', 'LineWidth', 1.2);
plot(t, e, 'LineWidth', 1.2);
grid on;
xlabel('time [s]');
ylabel('signals');
legend('y(t)', 'r', 'e(t)=y(t)-r', 'Location', 'best');
title('Chapter 27 Lesson 1: Tracking formulation simulation');

% Optional Simulink construction:
% The Simulink interpretation is the deviation model
%
%   x_tilde_dot = (A - B*K)*x_tilde
%   y_tilde     = C*x_tilde
%
% where x_tilde = x - x_bar and y_tilde = y - r.
if exist('simulink', 'file') == 4
    modelName = 'Chapter27_Lesson1_Simulink_Formulation';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end

    new_system(modelName);
    open_system(modelName);

    Acl = A - B*K;
    Bcl = zeros(2, 1);
    Ccl = C;
    Dcl = 0;

    add_block('simulink/Sources/Constant', [modelName '/zero input'], ...
        'Value', '0', 'Position', [80 80 130 110]);
    add_block('simulink/Continuous/State-Space', [modelName '/deviation state-space'], ...
        'A', mat2str(Acl), 'B', mat2str(Bcl), ...
        'C', mat2str(Ccl), 'D', mat2str(Dcl), ...
        'X0', mat2str(x0 - x_bar), 'Position', [190 65 340 125]);
    add_block('simulink/Sinks/Scope', [modelName '/tracking error scope'], ...
        'Position', [410 75 460 115]);

    add_line(modelName, 'zero input/1', 'deviation state-space/1');
    add_line(modelName, 'deviation state-space/1', 'tracking error scope/1');

    set_param(modelName, 'StopTime', '8');
    save_system(modelName);
    disp(['Optional Simulink model created: ' modelName '.slx']);
else
    disp('Simulink is not available; skipped optional Simulink model creation.');
end
