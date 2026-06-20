% Chapter11_Lesson3.m
% Controllability in companion controllable canonical form.
% Requires only base MATLAB for the custom controllability matrix.
% If Control System Toolbox is available, compare with ctrb(A,B).

clear; clc;

coeffs = [6 11 6];              % [a0 a1 a2] for p(s)=s^3+6s^2+11s+6
[A_c, B_c] = companion_pair(coeffs);
Q_c = controllability_matrix(A_c, B_c);

disp('A_c ='); disp(A_c);
disp('B_c ='); disp(B_c);
disp('Q_c = [B AB A^2B] ='); disp(Q_c);
fprintf('rank(Q_c) = %d\n', rank(Q_c));
fprintf('det(Q_c)  = %.6g\n', det(Q_c));

% Control System Toolbox comparison
if exist('ctrb', 'file') == 2
    fprintf('rank(ctrb(A_c,B_c)) = %d\n', rank(ctrb(A_c, B_c)));
end

% State-space object and transfer function (if Control System Toolbox exists)
if exist('ss', 'file') == 2
    C_c = [1 0 0];
    D_c = 0;
    sys = ss(A_c, B_c, C_c, D_c);
    disp('State-space model:');
    disp(sys);
end

% Simulink note:
% In Simulink, use a State-Space block with A=A_c, B=B_c, C=eye(n), D=zeros(n,1).
% Then drive it by a Step block and observe all states using a Scope.

function [A, B] = companion_pair(coeffs)
    n = numel(coeffs);
    A = zeros(n);
    if n > 1
        A(1:n-1, 2:n) = eye(n-1);
    end
    A(n, :) = -coeffs(:).';
    B = zeros(n, 1);
    B(n) = 1;
end

function Q = controllability_matrix(A, B)
    n = size(A, 1);
    Q = zeros(n);
    Ak = eye(n);
    for k = 1:n
        Q(:, k) = Ak * B;
        Ak = Ak * A;
    end
end
