% Chapter15_Lesson5.m
% Duality between observability and controllability Gramians.
% This script uses Control System Toolbox when available and also includes
% a numerical finite-horizon integral.

clear; clc;

A = [-1  2;
     -3 -4];
C = [1 0.5];
T = 3.0;

% Finite-horizon numerical integration.
N = 4000;
dt = T / N;
Wo = zeros(2,2);
WcDual = zeros(2,2);

for k = 0:N
    t = k * dt;
    E = expm(A * t);
    integrand_o = E' * C' * C * E;

    Edual = expm(A' * t);
    Bdual = C';
    integrand_c_dual = Edual * Bdual * Bdual' * Edual';

    weight = 1.0;
    if k == 0 || k == N
        weight = 0.5;
    end

    Wo = Wo + weight * integrand_o;
    WcDual = WcDual + weight * integrand_c_dual;
end

Wo = dt * Wo;
WcDual = dt * WcDual;

disp('Finite-horizon W_o(A,C):');
disp(Wo);

disp('Finite-horizon W_c(A'',C''):');
disp(WcDual);

disp('Finite-horizon Frobenius difference:');
disp(norm(Wo - WcDual, 'fro'));

% Infinite-horizon Lyapunov form for Hurwitz A:
% A' Wo + Wo A + C' C = 0
WoInf = lyap(A', C' * C);
WcDualInf = lyap(A', C' * C);

disp('Infinite-horizon W_o(A,C):');
disp(WoInf);

disp('Infinite-horizon W_c(A'',C''):');
disp(WcDualInf);

disp('Infinite-horizon Frobenius difference:');
disp(norm(WoInf - WcDualInf, 'fro'));

% Optional Control System Toolbox verification.
try
    sys = ss(A, [0;0], C, 0);
    sysDual = ss(A', C', eye(2), zeros(2,1));
    WoToolbox = gram(sys, 'o');
    WcDualToolbox = gram(sysDual, 'c');
    disp('Toolbox W_o(A,C):');
    disp(WoToolbox);
    disp('Toolbox W_c(A'',C''):');
    disp(WcDualToolbox);
catch ME
    disp('Control System Toolbox verification skipped:');
    disp(ME.message);
end

% Optional Simulink construction: creates a State-Space block for the dual system.
try
    modelName = 'Chapter15_Lesson5_Duality_Model';
    new_system(modelName);
    open_system(modelName);
    add_block('simulink/Continuous/State-Space', [modelName '/Dual_State_Space']);
    set_param([modelName '/Dual_State_Space'], ...
        'A', 'A''', ...
        'B', 'C''', ...
        'C', 'eye(2)', ...
        'D', 'zeros(2,1)');
    save_system(modelName);
    disp(['Created Simulink model: ' modelName '.slx']);
catch ME
    disp('Simulink model creation skipped:');
    disp(ME.message);
end
