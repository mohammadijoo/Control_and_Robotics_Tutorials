% Chapter23_Lesson5.m
% Numerical Sensitivity and Conditioning in SISO Pole Placement
%
% Requires: base MATLAB. Control System Toolbox is useful for place(), acker(),
% ctrb(), and ss(), but the Ackermann computation below is explicit.

clear; clc; close all;

A = [0 1 0;
     0 0 1;
    -2 -3 -4];

B = [0; 0; 1];

desired_poles = [-4 -5 -6];

C = controllability_local(A, B);
coeffs = poly(desired_poles);        % [1 alpha2 alpha1 alpha0]
K = ackermann_local(A, B, desired_poles);

M = A - B*K;

fprintf("Original coordinates\n");
fprintf("--------------------\n");
disp("Desired polynomial coefficients:");
disp(coeffs);
fprintf("cond(C) = %.6e\n", cond(C));
disp("K = ");
disp(K);
disp("Closed-loop eigenvalues:");
disp(eig(M));

[V, D, W] = eig(M); %#ok<ASGLU>
kappa = eig_condition_numbers(M);
disp("Eigenvalue condition numbers:");
disp(kappa.');

% Compare with MATLAB Control System Toolbox, if available:
if exist("acker", "file") == 2
    K_acker = acker(A, B, desired_poles);
    disp("MATLAB acker(A,B,p) = ");
    disp(K_acker);
end
if exist("place", "file") == 2
    K_place = place(A, B, desired_poles);
    disp("MATLAB place(A,B,p) = ");
    disp(K_place);
end

% Bad coordinate scaling: x = T z
Tbad = diag([1e-3 1 1e3]);
Abad = Tbad \ A * Tbad;
Bbad = Tbad \ B;
Cbad = controllability_local(Abad, Bbad);
Kz = ackermann_local(Abad, Bbad, desired_poles);
Kx = Kz / Tbad;

fprintf("\nBadly scaled design coordinates\n");
fprintf("-------------------------------\n");
fprintf("cond(C_bad) = %.6e\n", cond(Cbad));
disp("K_z = ");
disp(Kz);
disp("Equivalent K_x = ");
disp(Kx);
disp("Closed-loop eigenvalues using equivalent K_x:");
disp(eig(A - B*Kx));

% Perturbation experiment
rng(1);
N = 300;
eps_rel = 1e-7;
drift1 = perturbation_drift(A, B, K, eps_rel, N);
drift2 = perturbation_drift(A, B, Kx, eps_rel, N);

figure;
bar([log10(cond(C)), log10(cond(Cbad))]);
set(gca, "XTickLabel", ["original", "bad scaling"]);
ylabel("log10 cond(controllability matrix)");
title("Coordinate scaling can destroy numerical conditioning");

figure;
boxplot([drift1(:), drift2(:)], "Labels", ["original", "bad scaling"]);
ylabel("2-norm closed-loop pole drift");
title("Pole-placement sensitivity under small perturbations");


function C = controllability_local(A, B)
    n = size(A, 1);
    C = zeros(n, n);
    Ak = eye(n);
    for k = 1:n
        C(:, k) = Ak * B;
        Ak = A * Ak;
    end
end

function K = ackermann_local(A, B, poles)
    n = size(A, 1);
    C = controllability_local(A, B);
    coeffs = poly(poles);
    pA = zeros(n);
    for k = 1:length(coeffs)
        pA = pA * A + coeffs(k) * eye(n);
    end
    eT = zeros(1, n);
    eT(n) = 1;
    K = eT * (C \ pA);
end

function kappa = eig_condition_numbers(M)
    [V, D, W] = eig(M); %#ok<ASGLU>
    n = size(M, 1);
    kappa = zeros(n, 1);
    for i = 1:n
        v = V(:, i);
        w = W(:, i);
        kappa(i) = norm(w) * norm(v) / abs(w' * v);
    end
end

function drifts = perturbation_drift(A, B, K, eps_rel, N)
    lam0 = sort(eig(A - B*K));
    drifts = zeros(N, 1);
    for i = 1:N
        dA = randn(size(A));
        dB = randn(size(B));
        dA = dA * (eps_rel * max(1, norm(A, 2)) / norm(dA, 2));
        dB = dB * (eps_rel * max(1, norm(B, 2)) / norm(dB, 2));
        lam = sort(eig((A + dA) - (B + dB)*K));
        drifts(i) = norm(lam - lam0, 2);
    end
end
