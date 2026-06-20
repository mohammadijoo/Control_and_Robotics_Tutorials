% Chapter25_Lesson3.m
%
% Trade-offs in state-feedback design:
% speed of response vs. control effort vs. sensitivity.
%
% MATLAB requirements:
%   Control System Toolbox for place, ss, initial, lyap.
% Optional:
%   Simulink for the final programmatic model section.

clear; clc; close all;

A = [0 1; -2 -0.4];
B = [0; 1];
C = [1 0];
D = 0;
x0 = [1; 0];

poleSets = {
    'slow',   [-1+1i, -1-1i];
    'medium', [-3+3i, -3-3i];
    'fast',   [-6+6i, -6-6i]
};

fprintf('State-feedback trade-off table\n');
fprintf('%10s%12s%14s%14s%16s\n', 'case', 'speed', 'norm(K)', 'J_u(x0)', 'eig(A-BK)');

figure; hold on; grid on;
title('Faster poles reduce settling time');
xlabel('time (s)'); ylabel('state x_1');

figure; hold on; grid on;
title('Faster poles require larger input');
xlabel('time (s)'); ylabel('control input u');

for i = 1:size(poleSets, 1)
    name = poleSets{i, 1};
    poles = poleSets{i, 2};

    K = place(A, B, poles);
    Ac = A - B*K;

    % P_u solves Ac' P + P Ac + K'K = 0.
    P_u = lyap(Ac', K'*K);
    J_u = x0' * P_u * x0;

    eigVals = eig(Ac);
    speed = -max(real(eigVals));

    fprintf('%10s%12.4f%14.4f%14.4f    [%8.3f%+8.3fi, %8.3f%+8.3fi]\n', ...
        name, speed, norm(K, 2), J_u, ...
        real(eigVals(1)), imag(eigVals(1)), real(eigVals(2)), imag(eigVals(2)));

    sysCL = ss(Ac, B, C, D);
    t = linspace(0, 10, 2000);
    [y, tOut, x] = initial(sysCL, x0, t);
    u = -(K * x')';

    figure(1); plot(tOut, x(:, 1), 'DisplayName', name);
    figure(2); plot(tOut, u, 'DisplayName', name);
end

figure(1); legend('Location', 'best');
figure(2); legend('Location', 'best');

% Sensitivity experiment: perturb A and B and inspect closed-loop poles.
rng(7);
epsLevel = 0.02;
samples = 200;

for i = 2:3
    name = poleSets{i, 1};
    poles = poleSets{i, 2};
    K = place(A, B, poles);
    eigCloud = zeros(samples, 2);

    for s = 1:samples
        dA = epsLevel * randn(size(A));
        dB = epsLevel * randn(size(B));
        eigCloud(s, :) = eig((A + dA) - (B + dB)*K).';
    end

    fprintf('eigenvalue cloud std for %s design: %.5f\n', name, std(eigCloud(:)));
end

% Optional Simulink sketch:
% This creates a minimal closed-loop state-space model if Simulink is installed.
if exist('simulink', 'file') == 4
    model = 'Chapter25_Lesson3_Simulink';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);

    % Use the medium design for the Simulink block.
    K = place(A, B, [-3+3i, -3-3i]);
    Ac = A - B*K;

    add_block('simulink/Continuous/State-Space', [model '/ClosedLoopPlant'], ...
        'A', mat2str(Ac), 'B', mat2str(B), 'C', mat2str(C), 'D', mat2str(D), ...
        'X0', mat2str(x0), 'Position', [140 100 280 160]);

    add_block('simulink/Sources/Constant', [model '/ZeroInput'], ...
        'Value', '0', 'Position', [40 115 90 145]);

    add_block('simulink/Sinks/Scope', [model '/Scope'], ...
        'Position', [340 105 390 155]);

    add_line(model, 'ZeroInput/1', 'ClosedLoopPlant/1');
    add_line(model, 'ClosedLoopPlant/1', 'Scope/1');

    set_param(model, 'StopTime', '10');
    save_system(model);
    fprintf('Created optional Simulink model: %s.slx\n', model);
end
