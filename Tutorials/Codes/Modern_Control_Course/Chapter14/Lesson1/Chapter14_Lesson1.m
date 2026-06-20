% Chapter14_Lesson1.m
%
% Kalman observability matrix and rank condition.
% This script uses only basic MATLAB. If Control System Toolbox is available,
% it also compares the result with obsv(A,C).

clear; clc;

A1 = [0 1; 0 0];
C1 = [1 0];

A2 = [0 0; 0 -2];
C2 = [1 0];

analyzeObservability(A1, C1, "Example 1: observable double integrator");
analyzeObservability(A2, C2, "Example 2: unobservable second mode");

% Control System Toolbox comparison:
if exist("obsv", "file") == 2
    fprintf("\nControl System Toolbox check for Example 1:\n");
    disp(obsv(A1, C1));
end

% Simulink workflow note:
% 1. Put A, B, C, D into a State-Space block.
% 2. Linearize the model or use the same matrices directly.
% 3. Run O = obsv(A,C); rank(O).
% Observability of the initial state depends on A and C, not on B or D.

function O = observabilityMatrix(A, C)
    n = size(A, 1);
    O = [];
    Ak = eye(n);
    for k = 0:n-1
        O = [O; C * Ak]; %#ok<AGROW>
        Ak = Ak * A;
    end
end

function analyzeObservability(A, C, name)
    O = observabilityMatrix(A, C);
    s = svd(O);
    tol = max(size(O)) * eps(max(s));
    r = sum(s > tol);

    fprintf("\n%s\n", name);
    fprintf("%s\n", repmat("-", 1, strlength(name)));
    fprintf("A =\n"); disp(A);
    fprintf("C =\n"); disp(C);
    fprintf("Observability matrix O_n =\n"); disp(O);
    fprintf("Singular values =\n"); disp(s.');
    fprintf("Tolerance = %.4e\n", tol);
    fprintf("rank(O_n) = %d of n = %d\n", r, size(A,1));
    fprintf("Observable? %d\n", r == size(A,1));

    if r < size(A,1)
        fprintf("Basis for unobservable initial-state directions:\n");
        disp(null(O));
    end
end
