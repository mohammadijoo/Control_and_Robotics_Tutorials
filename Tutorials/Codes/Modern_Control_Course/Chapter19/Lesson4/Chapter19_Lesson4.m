% Chapter19_Lesson4.m
% Identification of a minimal realization via sequential Kalman decomposition.
% Requires Control System Toolbox for ctrb, obsv, ss, tf.

clear; clc;

% Transparent nonminimal realization.
Aco = [0 1; -2 -3];
Bco = [0; 1];
Cco = [1 0];

A = blkdiag(Aco, -4, -5);
B = [0; 1; 1; 0];
C = [1 0 0 2];
D = 0;

[Amin, Bmin, Cmin, Dmin, info] = kalman_minimal_decomposition(A, B, C, D);

disp('rank controllability ='); disp(info.reachable_rank);
disp('rank observability after reachable reduction ='); disp(info.observable_rank_after_reachable);
disp('Amin ='); disp(Amin);
disp('Bmin ='); disp(Bmin);
disp('Cmin ='); disp(Cmin);

sys_full = ss(A, B, C, D);
sys_min  = ss(Amin, Bmin, Cmin, Dmin);

disp('Full transfer function:');
tf(sys_full)
disp('Minimal transfer function:');
tf(sys_min)

% Optional comparison with MATLAB minreal.
disp('MATLAB minreal result:');
minreal(sys_full)

% Simulink note:
% The same matrices can be placed in a State-Space block. The reduced matrices
% Amin, Bmin, Cmin, Dmin define an equivalent State-Space block with fewer states.

function [Amin, Bmin, Cmin, Dmin, info] = kalman_minimal_decomposition(A, B, C, D)
    n = size(A, 1);

    % Step 1: reachable reduction.
    Wc = ctrb(A, B);
    R = orth(Wc);
    rc = size(R, 2);
    N = null(R');
    Tc = [R N];

    Ac = Tc' * A * Tc;
    Bc = Tc' * B;
    Cc = C * Tc;

    Ar = Ac(1:rc, 1:rc);
    Br = Bc(1:rc, :);
    Cr = Cc(:, 1:rc);

    % Step 2: observable reduction of the reachable subsystem.
    Wo = obsv(Ar, Cr);
    ro = rank(Wo);
    Nobs = null(Wo);
    Ocomp = orth(Wo');
    To = [Nobs Ocomp];

    Ao = To' * Ar * To;
    Bo = To' * Br;
    Co = Cr * To;

    n_unobs = rc - ro;
    Amin = Ao(n_unobs+1:end, n_unobs+1:end);
    Bmin = Bo(n_unobs+1:end, :);
    Cmin = Co(:, n_unobs+1:end);
    Dmin = D;

    info.reachable_rank = rc;
    info.observable_rank_after_reachable = ro;
    info.T_reachable = Tc;
    info.T_observable = To;
    info.A_reachable = Ar;
    info.B_reachable = Br;
    info.C_reachable = Cr;
end
