% Chapter14_Lesson2.m
% PBH Test for Observability and optional Simulink State-Space model creation.
%
% System:
%   x_dot = A x + B u
%   y     = C x + D u
%
% Observability depends only on (A, C).

clear; clc;

A1 = [0 1; -2 -3];
C1 = [1 0];

A2 = [-1 0; 0 -2];
C2 = [1 0];

tol = 1e-9;

fprintf('================ Example 1 ================\n');
pbh_observability_report(A1, C1, tol);

fprintf('================ Example 2 ================\n');
pbh_observability_report(A2, C2, tol);

% Optional: create a minimal Simulink model with a State-Space block.
% This requires Simulink. The model is for simulation; the PBH test itself is
% algebraic and is implemented above.
if exist('simulink', 'file') == 4
    model = 'Chapter14_Lesson2_PBH_Observability_Model';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);

    B = [0; 1];
    D = 0;
    assignin('base', 'A_sim', A1);
    assignin('base', 'B_sim', B);
    assignin('base', 'C_sim', C1);
    assignin('base', 'D_sim', D);

    add_block('simulink/Sources/Step', [model '/Step Input'], ...
        'Position', [80 90 120 120]);
    add_block('simulink/Continuous/State-Space', [model '/State-Space Plant'], ...
        'A', 'A_sim', 'B', 'B_sim', 'C', 'C_sim', 'D', 'D_sim', ...
        'Position', [180 75 330 135]);
    add_block('simulink/Sinks/Scope', [model '/Output Scope'], ...
        'Position', [400 85 440 125]);
    add_line(model, 'Step Input/1', 'State-Space Plant/1');
    add_line(model, 'State-Space Plant/1', 'Output Scope/1');
    save_system(model);
    fprintf('Created Simulink model: %s.slx\n', model);
else
    fprintf('Simulink not available; algebraic PBH test completed.\n');
end

function pbh_observability_report(A, C, tol)
    n = size(A, 1);
    O = observability_matrix(A, C);
    rankO = rank(O, tol);
    fprintf('Rank of observability matrix = %d of %d\n', rankO, n);

    lambdas = eig(A);
    observable = true;
    for k = 1:length(lambdas)
        lambda = lambdas(k);
        M = [lambda*eye(n) - A; C];
        r = rank(M, tol);
        passed = (r == n);
        observable = observable && passed;
        fprintf('lambda = %.6f%+.6fi, PBH rank = %d, passed = %d\n', ...
            real(lambda), imag(lambda), r, passed);
    end
    fprintf('Observable by PBH = %d\n', observable);
end

function O = observability_matrix(A, C)
    n = size(A, 1);
    O = [];
    for k = 0:n-1
        O = [O; C * (A^k)]; %#ok<AGROW>
    end
end
