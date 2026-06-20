% Chapter8_Lesson2.m
% Modern Control — Chapter 8, Lesson 2
% Semigroup Property and Inverse of the State Transition Matrix Phi(t)
%
% This script verifies Phi(t+s)=Phi(t)Phi(s) and inv(Phi(t))=Phi(-t).
% It also contains an optional Simulink construction section.

clear; clc;

A = [0 1; -4 -0.8];
t = 0.7;
s = 1.3;
x0 = [1; -0.25];

Phi_t = expm(A*t);
Phi_s = expm(A*s);
Phi_t_plus_s = expm(A*(t+s));
Phi_minus_t = expm(-A*t);

semigroup_error = norm(Phi_t_plus_s - Phi_t*Phi_s, 'fro');
inverse_error = norm(inv(Phi_t) - Phi_minus_t, 'fro');
identity_error = norm(Phi_t*Phi_minus_t - eye(size(A)), 'fro');

x_direct = Phi_t_plus_s*x0;
x_two_step = Phi_t*(Phi_s*x0);
state_error = norm(x_direct - x_two_step);

disp('Phi(t) ='); disp(Phi_t);
disp('Phi(s) ='); disp(Phi_s);
disp('Phi(t+s) ='); disp(Phi_t_plus_s);
fprintf('Semigroup error = %.3e\n', semigroup_error);
fprintf('Inverse error = %.3e\n', inverse_error);
fprintf('Identity error = %.3e\n', identity_error);
fprintf('State two-step propagation error = %.3e\n', state_error);

% Optional Simulink model generation:
% The model contains a continuous-time State-Space block with zero input.
% It is a vehicle for visualizing the homogeneous evolution xdot = A x.
try
    modelName = 'Chapter8_Lesson2_Simulink';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/Constant', [modelName '/ZeroInput'], ...
        'Value', '0', 'Position', [80 80 120 110]);
    add_block('simulink/Continuous/State-Space', [modelName '/StateSpace'], ...
        'A', mat2str(A), 'B', '[0;0]', 'C', 'eye(2)', 'D', '[0;0]', ...
        'X0', mat2str(x0), 'Position', [180 65 300 125]);
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
        'Position', [360 70 420 120]);
    add_line(modelName, 'ZeroInput/1', 'StateSpace/1');
    add_line(modelName, 'StateSpace/1', 'Scope/1');
    set_param(modelName, 'StopTime', num2str(t+s));

    fprintf('Created Simulink model: %s\n', modelName);
    fprintf('Run sim(modelName) in MATLAB to visualize x(t).\n');
catch ME
    fprintf('Simulink model generation skipped: %s\n', ME.message);
end
