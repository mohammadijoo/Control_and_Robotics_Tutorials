% Chapter15_Lesson1.m
% Observability Gramian for continuous-time LTI systems.
%
% Libraries/toolboxes:
%   Base MATLAB: expm, integral, eig
%   Control System Toolbox: gram, ss, lyap
%   Simulink: optional zero-input output simulation model
%
% System:
%   x_dot = A*x, y = C*x

clear; clc;

A = [0 1; -2 -3];
B = [0; 0];      % not used for observability, included for ss object
C = [1 0];
D = 0;

T = 4.0;
Q = C' * C;

% Finite-horizon Gramian by numerical quadrature
integrand = @(t) expm(A' * t) * Q * expm(A * t);
Wo_T = integral(integrand, 0, T, 'ArrayValued', true);
Wo_T = 0.5 * (Wo_T + Wo_T');

disp('Finite-horizon observability Gramian Wo(T):');
disp(Wo_T);
disp('Eigenvalues of Wo(T):');
disp(eig(Wo_T));

% Observability matrix and rank
O = obsv(A, C);
disp('Observability matrix:');
disp(O);
fprintf('rank(O) = %d\n', rank(O));

% Infinite-horizon Gramian for Hurwitz A:
% lyap(A',Q) solves A'*W + W*A + Q = 0.
Wo_inf = lyap(A', Q);
Wo_inf = 0.5 * (Wo_inf + Wo_inf');

disp('Infinite-horizon observability Gramian:');
disp(Wo_inf);
disp('Eigenvalues of Wo_inf:');
disp(eig(Wo_inf));

% Same infinite-horizon result using the Control System Toolbox gram command.
sys = ss(A, B, C, D);
Wo_control_toolbox = gram(sys, 'o');
disp('Observability Gramian from gram(sys,''o''):');
disp(Wo_control_toolbox);

x0 = [1; -0.5];
energy_T = x0' * Wo_T * x0;
fprintf('Output energy over [0,T] = %.10f\n', energy_T);

% Optional Simulink model: zero-input state-space block with initial condition.
% This block simulates y(t)=C*x(t) for the chosen x0.
createSimulinkModel = false;

if createSimulinkModel
    model = 'Chapter15_Lesson1_ZeroInputOutput';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);

    add_block('simulink/Continuous/State-Space', [model '/ZeroInputStateSpace']);
    set_param([model '/ZeroInputStateSpace'], ...
        'A', mat2str(A), ...
        'B', mat2str(B), ...
        'C', mat2str(C), ...
        'D', mat2str(D), ...
        'X0', mat2str(x0));

    add_block('simulink/Sources/Constant', [model '/ZeroInput']);
    set_param([model '/ZeroInput'], 'Value', '0');

    add_block('simulink/Sinks/Scope', [model '/OutputScope']);

    add_line(model, 'ZeroInput/1', 'ZeroInputStateSpace/1');
    add_line(model, 'ZeroInputStateSpace/1', 'OutputScope/1');

    set_param(model, 'StopTime', num2str(T));
    save_system(model);
end
