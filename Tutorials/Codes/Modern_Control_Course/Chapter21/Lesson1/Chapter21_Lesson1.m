% Chapter21_Lesson1.m
% Finite transmission zeros from the Rosenbrock system matrix.
% MATLAB functions used:
%   ss, tzero   : Control System Toolbox, if available
%   rank, svd   : core numerical linear algebra
%   syms, det   : Symbolic Math Toolbox, if available
%
% Simulink section:
%   Creates a simple State-Space block model if Simulink is installed.

clear; clc;

A = [0 1; -2 -3];
B = [0; 1];
C = [4 1];
D = 0;

fprintf('System: G(s) = (s + 4)/(s^2 + 3s + 2)\n');

% Control System Toolbox route
if exist('ss', 'file') == 2 && exist('tzero', 'file') == 2
    sys = ss(A, B, C, D);
    z = tzero(sys);
    disp('Transmission zeros from tzero(sys):');
    disp(z);
else
    disp('Control System Toolbox tzero not found; using Rosenbrock rank check.');
end

% Rosenbrock rank check at z = -4
z0 = -4;
Rz = [z0*eye(size(A,1)) - A, -B; C, D];
disp('R(-4) =');
disp(Rz);
fprintf('rank R(-4) = %d\n', rank(Rz));
fprintf('normal rank for this SISO square case = %d\n', size(A,1) + size(B,2));

% Symbolic determinant route
if exist('syms', 'file') == 2
    syms s
    R = [s*eye(size(A,1)) - A, -B; C, D];
    zeroPolynomial = factor(det(R));
    disp('det R(s) =');
    disp(zeroPolynomial);
    disp('roots(det R(s)) =');
    disp(double(solve(zeroPolynomial == 0, s)));
else
    disp('Symbolic Math Toolbox not found; skipping symbolic determinant.');
end

% Verify zero-output exponential trajectory
x0 = [1; -4];
u0 = 6;
stateResidual = z0*x0 - A*x0 - B*u0;
outputResidual = C*x0 + D*u0;
disp('state residual z*x0 - A*x0 - B*u0 =');
disp(stateResidual);
disp('output residual C*x0 + D*u0 =');
disp(outputResidual);

% Optional Simulink model generation
if exist('simulink', 'file') == 2
    modelName = 'Chapter21_Lesson1_Simulink';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/Step', [modelName '/StepInput'], ...
        'Position', [60 80 100 110]);
    add_block('simulink/Continuous/State-Space', [modelName '/StateSpacePlant'], ...
        'A', mat2str(A), 'B', mat2str(B), 'C', mat2str(C), 'D', mat2str(D), ...
        'Position', [170 70 300 120]);
    add_block('simulink/Sinks/Scope', [modelName '/OutputScope'], ...
        'Position', [380 75 430 115]);

    add_line(modelName, 'StepInput/1', 'StateSpacePlant/1');
    add_line(modelName, 'StateSpacePlant/1', 'OutputScope/1');

    save_system(modelName);
    disp(['Simulink model saved as ', modelName, '.slx']);
else
    disp('Simulink not found; skipping model generation.');
end
