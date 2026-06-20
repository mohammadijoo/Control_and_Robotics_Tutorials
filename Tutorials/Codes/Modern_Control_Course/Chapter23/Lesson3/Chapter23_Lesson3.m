% Chapter23_Lesson3.m
% Ackermann's formula for SISO pole placement.
% The Control System Toolbox has acker(A,B,poles), but this script also
% implements the formula explicitly.

clear; clc;

A = [0 1; -2 -3];
B = [0; 1];
desired_poles = [-2+2i, -2-2i];

K = ackermann_gain(A, B, desired_poles);
disp('K from custom Ackermann formula:');
disp(K);

Acl = A - B*K;
disp('Closed-loop eigenvalues:');
disp(eig(Acl));

if exist('acker', 'file') == 2
    disp('K from MATLAB acker:');
    disp(acker(A, B, desired_poles));
end

% Simulink note:
% Use a State-Space block with A, B, C=eye(size(A)), D=zeros(size(B)).
% Feed the measured state vector x into a Gain block with gain -K, then feed
% that signal into the State-Space block input. Scope x and u to verify decay.

function Ctrb = controllability_matrix(A, B)
    n = size(A, 1);
    Ctrb = zeros(n, n);
    for k = 1:n
        Ctrb(:, k) = A^(k-1) * B;
    end
end

function phiA = matrix_polynomial(A, coeff)
    % coeff = [1 alpha_{n-1} ... alpha_0]
    n = size(A, 1);
    phiA = A^n;
    for i = 2:length(coeff)
        power = n - (i - 1);
        if power == 0
            term = eye(n);
        else
            term = A^power;
        end
        phiA = phiA + coeff(i) * term;
    end
end

function K = ackermann_gain(A, B, desired_poles)
    n = size(A, 1);
    Ctrb = controllability_matrix(A, B);
    if rank(Ctrb) ~= n
        error('System is not controllable, so arbitrary pole placement is impossible.');
    end
    coeff = poly(desired_poles);
    phiA = matrix_polynomial(A, coeff);
    eT = zeros(1, n);
    eT(end) = 1;
    K = eT / Ctrb * phiA;
    K = real(K);
end
