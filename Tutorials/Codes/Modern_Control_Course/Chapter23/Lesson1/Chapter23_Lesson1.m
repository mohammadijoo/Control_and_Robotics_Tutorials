% Chapter23_Lesson1.m
% Formulating the SISO pole-placement problem in MATLAB/Simulink.
% This script checks controllability, constructs the desired polynomial,
% performs second-order companion-form coefficient matching, and builds a
% simple Simulink closed-loop state-space model if Simulink is available.

clear; clc; close all;

A = [0 1; -2 -3];
b = [0; 1];
C = [1 0];
D = 0;

Ctrb = ctrb(A, b);
fprintf('rank(Ctrb) = %d\n', rank(Ctrb));
fprintf('controllable = %d\n', rank(Ctrb) == size(A, 1));

desired_poles = [-4 -5];
open_poly = poly(A);          % [1 a1 a0]
desired_poly = poly(desired_poles); % [1 alpha1 alpha0]

fprintf('open-loop polynomial coefficients [1 a1 a0]:\n');
disp(open_poly);
fprintf('desired polynomial coefficients [1 alpha1 alpha0]:\n');
disp(desired_poly);

% For A = [0 1; -a0 -a1], b = [0; 1], K = [alpha0-a0, alpha1-a1].
a0 = open_poly(3);
a1 = open_poly(2);
alpha0 = desired_poly(3);
alpha1 = desired_poly(2);
K_manual = [alpha0 - a0, alpha1 - a1];

Acl = A - b * K_manual;
fprintf('K_manual =\n');
disp(K_manual);
fprintf('closed-loop eigenvalues from manual coefficient matching:\n');
disp(eig(Acl));
fprintf('closed-loop polynomial coefficients:\n');
disp(poly(Acl));

% MATLAB Control System Toolbox comparison.
% The command place(A,b,desired_poles) is a library implementation. It is shown
% here only as a verification tool; later lessons derive full formulas.
if exist('place', 'file') == 2
    K_place = place(A, b, desired_poles);
    fprintf('K_place =\n');
    disp(K_place);
end

% Closed-loop state-space response to an initial condition.
sys_cl = ss(Acl, b, C, D);
t = linspace(0, 4, 400);
u = zeros(size(t));
x0 = [1; 0];
[y, t_out, x] = lsim(sys_cl, u, t, x0);

figure;
plot(t_out, x(:,1), 'LineWidth', 1.5); hold on;
plot(t_out, x(:,2), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('States');
legend('x_1', 'x_2');
title('Closed-loop state response after pole placement formulation');

% Optional Simulink model generation.
if exist('new_system', 'file') == 2
    modelName = 'Chapter23_Lesson1_Simulink_Model';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/Constant', [modelName '/zero input'], ...
        'Value', '0', 'Position', [80 90 130 120]);
    add_block('simulink/Continuous/State-Space', [modelName '/ClosedLoopStateSpace'], ...
        'A', 'Acl', 'B', 'b', 'C', 'C', 'D', 'D', ...
        'X0', '[1;0]', 'Position', [190 75 340 135]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
        'Position', [420 82 470 128]);

    add_line(modelName, 'zero input/1', 'ClosedLoopStateSpace/1');
    add_line(modelName, 'ClosedLoopStateSpace/1', 'Scope/1');
    set_param(modelName, 'StopTime', '4');
    save_system(modelName);
    fprintf('Simulink model saved as %s.slx\n', modelName);
end