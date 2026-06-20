% Chapter11_Lesson1.m
% Kalman controllability matrix and rank condition in MATLAB.
% The script uses a from-scratch implementation and also shows the Control System Toolbox call.

clear; clc;

A = [0 1 0;
     0 0 1;
    -6 -11 -6];

B = [0;
     0;
     1];

Ck = kalmanControllabilityMatrix(A, B);
r = rank(Ck);

disp('Kalman controllability matrix C_K =');
disp(Ck);
fprintf('rank(C_K) = %d\n', r);
fprintf('controllable = %s\n', string(r == size(A, 1)));

% If the Control System Toolbox is available, ctrb(A,B) gives the same matrix.
if exist('ctrb', 'file') == 2
    Ck_toolbox = ctrb(A, B);
    fprintf('norm(Ck - ctrb(A,B), fro) = %.3e\n', norm(Ck - Ck_toolbox, 'fro'));
end

% Optional Simulink interpretation:
% Build an integrator-chain realization with State-Space block parameters
% A, B, C = eye(size(A)), D = zeros(size(A,1), size(B,2)).
% The rank test checks whether the input port has authority over all internal states.

function Ck = kalmanControllabilityMatrix(A, B)
    n = size(A, 1);
    if size(A, 2) ~= n
        error('A must be square.');
    end
    if size(B, 1) ~= n
        error('B must have the same number of rows as A.');
    end

    Ck = [];
    Ak = eye(n);
    for k = 0:n-1
        Ck = [Ck, Ak * B]; %#ok<AGROW>
        Ak = A * Ak;
    end
end
