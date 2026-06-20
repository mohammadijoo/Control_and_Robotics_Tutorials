% Chapter20_Lesson1.m
% Minimal realization tests for continuous-time LTI systems.
% Requires: MATLAB. Control System Toolbox is optional but recommended for ss,
% tf, minreal, ctrb, and obsv.
%
% The transfer matrix is G(s) = C*(sI - A)^(-1)*B + D.
% A realization is minimal iff (A,B) is reachable and (C,A) is observable.

clear; clc;

A_nonmin = [-1 0;
             0 -2];
B_nonmin = [1;
            1];
C_nonmin = [0 1];
D_nonmin = 0;

A_min = -2;
B_min = 1;
C_min = 1;
D_min = 0;

fprintf('\nTwo-state nonminimal realization\n');
reportMinimality(A_nonmin, B_nonmin, C_nonmin);

fprintf('\nOne-state minimal realization\n');
reportMinimality(A_min, B_min, C_min);

% Optional Control System Toolbox verification.
if exist('ss', 'file') == 2
    sys_nonmin = ss(A_nonmin, B_nonmin, C_nonmin, D_nonmin);
    sys_min = ss(A_min, B_min, C_min, D_min);

    fprintf('\nTransfer functions:\n');
    disp(tf(sys_nonmin));
    disp(tf(sys_min));

    fprintf('\nMATLAB minreal result for the nonminimal model:\n');
    disp(minreal(sys_nonmin));

    % Simulink workflow note:
    % 1. Build or open a Simulink model.
    % 2. Define linear analysis input/output points.
    % 3. Use linearize(modelName) to obtain an ss object.
    % 4. Apply minreal(sys) and compare rank(ctrb(sys)) and rank(obsv(sys)).
end

function R = reachabilityMatrix(A, B)
    n = size(A, 1);
    R = [];
    Ak = eye(n);
    for k = 1:n
        R = [R, Ak*B]; %#ok<AGROW>
        Ak = A*Ak;
    end
end

function O = observabilityMatrix(A, C)
    n = size(A, 1);
    O = [];
    Ak = eye(n);
    for k = 1:n
        O = [O; C*Ak]; %#ok<AGROW>
        Ak = Ak*A;
    end
end

function reportMinimality(A, B, C)
    n = size(A, 1);
    R = reachabilityMatrix(A, B);
    O = observabilityMatrix(A, C);

    fprintf('rank(R_n) = %d of n = %d\n', rank(R), n);
    fprintf('rank(O_n) = %d of n = %d\n', rank(O), n);
    fprintf('minimal?  = %s\n', string(rank(R) == n && rank(O) == n));
end
