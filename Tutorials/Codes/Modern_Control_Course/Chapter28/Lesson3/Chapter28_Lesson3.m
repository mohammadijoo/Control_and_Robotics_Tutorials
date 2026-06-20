% Chapter28_Lesson3.m
% Modern Control — Chapter 28, Lesson 3
% State and control weighting matrices Q and R as performance descriptors.
%
% This script demonstrates:
% 1. Bryson-style weight construction.
% 2. Finite-horizon weighted cost for a closed-loop state trajectory.
% 3. Coordinate-transformation consistency.
% 4. Optional programmatic creation of a small Simulink model.

clear; clc;

A = [0 1; -2 -0.4];
B = [0; 1];
K = [3.0 2.2];
x0 = [1; 0];

% Bryson-style tolerances.
xMax = [1; 2];
uMax = 0.5;
Q = diag(1 ./ (xMax.^2));
R = 1 / (uMax^2);

fprintf('eig(Q) = '); disp(eig((Q + Q')/2)');
fprintf('eig(R) = %.6f\n', R);

Acl = A - B*K;
[t, X] = ode45(@(t,x) Acl*x, [0 8], x0);
U = -(K * X')';

stateTerms = sum((X*Q).*X, 2);
inputTerms = (U.^2) * R;
J = trapz(t, stateTerms + inputTerms);
fprintf('Finite-horizon weighted cost J_T = %.6f\n', J);

% Coordinate transformation x = T z. Then Qz = T' Q T.
T = [2 0.5; 0 1.5];
Z = (T \ X')';
Qz = T' * Q * T;
Jx = trapz(t, sum((X*Q).*X, 2));
Jz = trapz(t, sum((Z*Qz).*Z, 2));
fprintf('State cost in x-coordinates = %.6f\n', Jx);
fprintf('State cost in z-coordinates = %.6f\n', Jz);
fprintf('Coordinate consistency error = %.3e\n', abs(Jx - Jz));

% Optional Simulink model: run only when Simulink is available.
if license('test', 'Simulink')
    modelName = 'Chapter28_Lesson3_Simulink_QR_Cost';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/Constant', [modelName '/x_vector'], ...
        'Value', '[1; 0]', 'Position', [80 80 150 120]);
    add_block('simulink/Math Operations/Gain', [modelName '/Q_gain'], ...
        'Gain', mat2str(Q), 'Multiplication', 'Matrix(K*u)', 'Position', [220 70 300 130]);
    add_block('simulink/Math Operations/Dot Product', [modelName '/xTQx'], ...
        'Position', [380 70 460 130]);
    add_block('simulink/Sinks/Display', [modelName '/Display_xTQx'], ...
        'Position', [540 80 650 120]);

    add_line(modelName, 'x_vector/1', 'Q_gain/1');
    add_line(modelName, 'x_vector/1', 'xTQx/1');
    add_line(modelName, 'Q_gain/1', 'xTQx/2');
    add_line(modelName, 'xTQx/1', 'Display_xTQx/1');

    save_system(modelName);
    fprintf('Created optional Simulink model: %s.slx\n', modelName);
end
