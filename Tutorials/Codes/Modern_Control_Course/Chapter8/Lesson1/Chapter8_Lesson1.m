% Chapter8_Lesson1.m
% Modern Control — Chapter 8, Lesson 1: Definition of the State Transition Matrix
%
% This script:
%   1) Computes Phi(t,t0) = expm(A*(t-t0))
%   2) Propagates x(t) = Phi x0 for xdot = A x
%   3) Verifies against numerical integration (ode45)
%   4) (Optional) Builds a simple Simulink model programmatically and simulates it
%
% Requirements:
%   - MATLAB base (expm, ode45)
%   - For ss/initial: Control System Toolbox (optional)
%   - For Simulink portion: Simulink (optional)

clear; clc;

A  = [0 1; -2 -3];
x0 = [1; -0.5];

t0 = 0.0;
t1 = 1.25;

% Analytic STM and state propagation
Phi = expm(A*(t1 - t0));
x_analytic = Phi*x0;

fprintf('A =\n'); disp(A);
fprintf('Phi(t1,t0) =\n'); disp(Phi);
fprintf('x_analytic =\n'); disp(x_analytic);

% Numerical verification using ode45
f = @(t,x) A*x;
opts = odeset('RelTol',1e-10,'AbsTol',1e-12);
[tt,xx] = ode45(f, [t0 t1], x0, opts);
x_numeric = xx(end,:).';
fprintf('x_numeric =\n'); disp(x_numeric);
fprintf('||x_analytic - x_numeric||_2 = %.3e\n', norm(x_analytic - x_numeric, 2));

% Optional: build and simulate a simple Simulink model (requires Simulink)
% The model simulates xdot = A x with zero input, outputting x to workspace.
try
    mdl = 'Chapter8_Lesson1_STM_Model';
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);

    add_block('simulink/Sources/Constant', [mdl '/u'], 'Value', '0');
    add_block('simulink/Continuous/State-Space', [mdl '/Plant']);
    add_block('simulink/Sinks/To Workspace', [mdl '/x_out'], 'VariableName', 'x_sim', 'SaveFormat', 'Array');

    set_param([mdl '/Plant'], 'A', mat2str(A), 'B', mat2str([0;0]), 'C', 'eye(2)', 'D', '0', 'X0', mat2str(x0));
    set_param(mdl, 'StopTime', num2str(t1));

    add_line(mdl, 'u/1', 'Plant/1');
    add_line(mdl, 'Plant/1', 'x_out/1');

    sim(mdl);

    % x_sim is an array: [t, x1, x2]
    x_end = x_sim(end, 2:3).';
    fprintf('x_simulink(end) =\n'); disp(x_end);
    fprintf('||x_simulink(end) - x_analytic||_2 = %.3e\n', norm(x_end - x_analytic, 2));

catch ME
    fprintf('\n[Simulink portion skipped] %s\n', ME.message);
end
