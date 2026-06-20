% Chapter18_Lesson3.m
% Dynamics associated with Jordan chains: polynomial-exponential terms.
%
% This script computes exp(Jt) from the nilpotent formula and compares it
% with MATLAB expm. It also optionally creates a Simulink model containing a
% State-Space block for the same Jordan block.

clear; clc; close all;

lambda = -0.40;
n = 3;
N = diag(ones(1, n - 1), 1);
J = lambda * eye(n) + N;
z0 = [1.0; -0.5; 2.0];

t = linspace(0, 12, 400);
Z = zeros(numel(t), n);

for k = 1:numel(t)
    E = jordanExponential(lambda, N, t(k));
    Z(k, :) = (E * z0).';
end

disp('Jordan block J = lambda I + N:');
disp(J);

disp('exp(Jt) from nilpotent formula at t = 2:');
E_formula = jordanExponential(lambda, N, 2.0);
disp(E_formula);

disp('Infinity-norm error versus expm(J*2):');
disp(norm(E_formula - expm(J * 2.0), inf));

figure;
plot(t, Z(:, 1), 'LineWidth', 1.5); hold on;
plot(t, Z(:, 2), 'LineWidth', 1.5);
plot(t, Z(:, 3), 'LineWidth', 1.5);
grid on;
xlabel('time t');
ylabel('Jordan-coordinate state');
title('Jordan-chain dynamics: polynomial-exponential terms');
legend('z1(t)', 'z2(t)', 'z3(t)', 'Location', 'best');

% Optional: create a Simulink model with a State-Space block.
% Uncomment the next line if Simulink is installed and licensed.
% createJordanChainSimulinkModel(lambda);

function E = jordanExponential(lambda, N, t)
    n = size(N, 1);
    S = eye(n);
    P = eye(n);
    for ell = 1:(n - 1)
        P = P * N;
        S = S + (t^ell / factorial(ell)) * P;
    end
    E = exp(lambda * t) * S;
end

function createJordanChainSimulinkModel(lambda)
    if ~license('test', 'Simulink')
        warning('Simulink license is not available.');
        return;
    end

    n = 3;
    N = diag(ones(1, n - 1), 1);
    J = lambda * eye(n) + N;
    B = zeros(n, 1);
    C = eye(n);
    D = zeros(n, 1);

    model = 'Chapter18_Lesson3_JordanChain';
    if bdIsLoaded(model)
        close_system(model, 0);
    end

    new_system(model);
    open_system(model);

    add_block('simulink/Continuous/State-Space', [model '/Jordan Chain State-Space']);
    set_param([model '/Jordan Chain State-Space'], ...
        'A', mat2str(J), ...
        'B', mat2str(B), ...
        'C', mat2str(C), ...
        'D', mat2str(D), ...
        'X0', '[1; -0.5; 2]');

    add_block('simulink/Sinks/Scope', [model '/Scope']);
    add_line(model, 'Jordan Chain State-Space/1', 'Scope/1');
    set_param(model, 'StopTime', '12');
    save_system(model);
end
