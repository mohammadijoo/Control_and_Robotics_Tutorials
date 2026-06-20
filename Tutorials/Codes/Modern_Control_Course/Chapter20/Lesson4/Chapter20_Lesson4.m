% Chapter20_Lesson4.m
% Basic algorithms for exact realization reduction.
% Uses MATLAB functions orth, null, rank. Control System Toolbox functions
% ctrb, obsv, ss, and minreal are also shown for comparison.

clear; clc;

A = diag([-1 -2 -3]);
B = [1; 0; 1];
C = [1 0 0];
D = 0;

R = local_ctrb(A,B);
O = local_obsv(A,C);
fprintf('rank(R) = %d of %d\n', rank(R), size(A,1));
fprintf('rank(O) = %d of %d\n', rank(O), size(A,1));

[Amin,Bmin,Cmin,Dmin] = exact_minimal_reduction(A,B,C,D);
disp('Amin ='); disp(Amin);
disp('Bmin ='); disp(Bmin);
disp('Cmin ='); disp(Cmin);
disp('Dmin ='); disp(Dmin);

% Optional Control System Toolbox comparison:
% sys = ss(A,B,C,D);
% sys_min = minreal(sys);
% disp(sys_min);

function R = local_ctrb(A,B)
    n = size(A,1);
    R = [];
    Apow = eye(n);
    for k = 1:n
        R = [R, Apow*B]; %#ok<AGROW>
        Apow = Apow*A;
    end
end

function O = local_obsv(A,C)
    n = size(A,1);
    O = [];
    Apow = eye(n);
    for k = 1:n
        O = [O; C*Apow]; %#ok<AGROW>
        Apow = Apow*A;
    end
end

function [Amin,Bmin,Cmin,Dmin] = exact_minimal_reduction(A,B,C,D)
    % Stage 1: reachable restriction.
    R = local_ctrb(A,B);
    Qr = orth(R);
    A1 = Qr'*A*Qr;
    B1 = Qr'*B;
    C1 = C*Qr;

    % Stage 2: observable quotient inside reachable part.
    O1 = local_obsv(A1,C1);
    Qun = null(O1);
    q = size(Qun,2);
    if q == 0
        Amin = A1; Bmin = B1; Cmin = C1; Dmin = D;
        return;
    end

    Qobs = null(Qun');
    T = [Qun, Qobs];
    Ahat = T'*A1*T;
    Bhat = T'*B1;
    Chat = C1*T;

    Amin = Ahat(q+1:end, q+1:end);
    Bmin = Bhat(q+1:end, :);
    Cmin = Chat(:, q+1:end);
    Dmin = D;
end
