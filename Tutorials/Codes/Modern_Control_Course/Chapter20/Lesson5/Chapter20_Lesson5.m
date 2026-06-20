% Chapter20_Lesson5.m
% Numerical issues in realization and model reduction.
% Requires Control System Toolbox for ss, gram, balreal, modred, minreal.
% Simulink section requires Simulink.

clear; clc; close all;

A = [-0.20  0.05  0.00  0.00;
      0.00 -1.00  0.10  0.00;
      0.00  0.00 -8.00  0.20;
      0.00  0.00  0.00 -20.0];

B = [1.0; 0.4; 0.05; 0.01];
C = [1.0 0.3 0.02 0.005];
D = 0;

sys = ss(A,B,C,D);

fprintf('Condition number of controllability matrix: %.6e\n', cond(ctrb(A,B)));
fprintf('Condition number of observability matrix: %.6e\n', cond(obsv(A,C)));

svC = svd(ctrb(A,B));
svO = svd(obsv(A,C));

disp('Controllability singular values:');
disp(svC.');

disp('Observability singular values:');
disp(svO.');

% MATLAB minreal uses tolerance logic to cancel weak pole-zero pairs.
sysMinDefault = minreal(sys);
sysMinTight   = minreal(sys, 1e-10);

disp('Default minreal order:');
disp(order(sysMinDefault));

disp('Tight tolerance minreal order:');
disp(order(sysMinTight));

% Balanced realization and model reduction.
[sysBal, hsv] = balreal(sys);
disp('Hankel singular values:');
disp(hsv.');

r = 2;
elim = (r+1):order(sysBal);
sysRed = modred(sysBal, elim, 'truncate');

fprintf('Reduced model order: %d\n', order(sysRed));
fprintf('Balanced truncation error upper bound: %.6e\n', 2*sum(hsv(r+1:end)));

figure;
bodemag(sys, sysRed);
grid on;
legend('Original','Balanced truncation r=2');
title('Chapter 20 Lesson 5: Numerical model reduction comparison');

% Optional Simulink block creation for side-by-side State-Space blocks.
% This programmatically creates a simple model with original and reduced systems.
modelName = 'Chapter20_Lesson5_Simulink';
if exist('simulink', 'file') == 4
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/Step', [modelName '/Step'], ...
        'Position', [40 100 70 130]);

    add_block('simulink/Continuous/State-Space', [modelName '/Original State-Space'], ...
        'A', 'A', 'B', 'B', 'C', 'C', 'D', 'D', ...
        'Position', [140 60 280 120]);

    Ar = sysRed.A; Br = sysRed.B; Cr = sysRed.C; Dr = sysRed.D; %#ok<NASGU>
    assignin('base','Ar',Ar);
    assignin('base','Br',Br);
    assignin('base','Cr',Cr);
    assignin('base','Dr',Dr);

    add_block('simulink/Continuous/State-Space', [modelName '/Reduced State-Space'], ...
        'A', 'Ar', 'B', 'Br', 'C', 'Cr', 'D', 'Dr', ...
        'Position', [140 160 280 220]);

    add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
        'Position', [370 90 400 180]);

    add_line(modelName, 'Step/1', 'Original State-Space/1');
    add_line(modelName, 'Step/1', 'Reduced State-Space/1');
    add_line(modelName, 'Original State-Space/1', 'Scope/1');
    add_line(modelName, 'Reduced State-Space/1', 'Scope/2');

    save_system(modelName);
    fprintf('Created Simulink model: %s.slx\n', modelName);
end
