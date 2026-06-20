% Chapter17_Lesson1.m
% Random Variables, Random Processes, and Stationarity Concepts
% MATLAB / Simulink implementation for System Dynamics (Chapter 17, Lesson 1)

clear; clc; close all;
rng(17);

%% Example 1: Random-phase harmonic process (WSS)
% X(t) = A*cos(omega*t + Theta), Theta ~ U(0, 2*pi)
A = 2.0;
omega = 3.0;
M = 2000;
t = linspace(0, 4, 401);
Nt = numel(t);

theta = 2*pi*rand(M,1);
X = zeros(M, Nt);
for m = 1:M
    X(m,:) = A*cos(omega*t + theta(m));
end

meanX = mean(X, 1);
varX  = var(X, 1, 1);   % population variance (normalize by N)

lags = [0, 10, 30, 60];
dt = t(2)-t(1);
fprintf('Random-phase harmonic process estimates:\n');
for L = lags
    if L == 0
        x1 = X;
        x2 = X;
    else
        x1 = X(:,1:end-L);
        x2 = X(:,1+L:end);
    end
    Rhat = mean(x1(:).*x2(:));
    tau = L*dt;
    Rth = 0.5*A^2*cos(omega*tau);
    fprintf('lag=%3d, tau=%7.4f, Rhat=%9.5f, Rtheory=%9.5f\n', L, tau, Rhat, Rth);
end

%% Example 2: Nonstationary process Y(t) = alpha*t + beta
alpha = 0.5 + 0.2*randn(M,1);
beta  = randn(M,1);
Y = alpha*t + beta;

meanY = mean(Y,1);
varY  = var(Y,1,1);

fprintf('\nNonstationary process estimates (time-varying moments):\n');
sel = [1, 101, 201, 401];
for idx = sel
    fprintf('t=%6.3f, mean=%9.5f, var=%9.5f\n', t(idx), meanY(idx), varY(idx));
end

%% Example 3: Discrete AR(1) process in stationary regime
% x[k] = a*x[k-1] + w[k], stationarity requires |a| < 1
a = 0.8;
sigma_w = 1.0;
K = 400;
Mar = 500;

sigma_x0 = sigma_w/sqrt(1-a^2); % invariant std
Xar = zeros(Mar, K);
Xar(:,1) = sigma_x0*randn(Mar,1);

for k = 2:K
    Xar(:,k) = a*Xar(:,k-1) + sigma_w*randn(Mar,1);
end

meanAR = mean(Xar,1);
varAR  = var(Xar,1,1);

fprintf('\nAR(1) stationary regime estimates:\n');
kSel = [1, 101, 201, 400];
for idx = kSel
    fprintf('k=%3d, mean=%9.5f, var=%9.5f\n', idx-1, meanAR(idx), varAR(idx));
end
fprintf('Theoretical stationary variance = %9.5f\n', sigma_w^2/(1-a^2));

%% Plot results
figure('Name','Chapter17 Lesson1 - Stationarity Concepts');
subplot(2,1,1);
plot(t, meanX, 'LineWidth', 1.2); hold on;
plot(t, zeros(size(t)), '--', 'LineWidth', 1.0);
grid on;
xlabel('t [s]'); ylabel('Ensemble Mean');
title('Random-phase harmonic process: estimated ensemble mean');
legend('Estimated','Theory mean = 0','Location','best');

subplot(2,1,2);
plot(t, varY, 'LineWidth', 1.2);
grid on;
xlabel('t [s]'); ylabel('Variance');
title('Nonstationary process Y(t)=alpha t + beta: variance depends on time');

%% Optional Simulink model creation (AR(1)-like recursion)
% This block programmatically creates a simple Simulink diagram illustrating
% a stochastic recursion x[k] = a*x[k-1] + w[k]. It requires Simulink.
try
    mdl = 'Chapter17_Lesson1_Simulink';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);

    add_block('simulink/Sources/Random Number', [mdl '/w'], ...
        'Position', [40 80 120 110]);
    add_block('simulink/Math Operations/Gain', [mdl '/sigma_w'], ...
        'Gain', '1.0', 'Position', [150 75 220 115]);
    add_block('simulink/Math Operations/Sum', [mdl '/Sum'], ...
        'Inputs', '++', 'Position', [280 70 310 120]);
    add_block('simulink/Math Operations/Gain', [mdl '/a'], ...
        'Gain', '0.8', 'Position', [350 130 420 170]);
    add_block('simulink/Discrete/Unit Delay', [mdl '/z^-1'], ...
        'Position', [460 125 520 175], 'SampleTime', '0.01');
    add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
        'Position', [560 70 620 110]);

    add_line(mdl, 'w/1', 'sigma_w/1');
    add_line(mdl, 'sigma_w/1', 'Sum/1');
    add_line(mdl, 'Sum/1', 'Scope/1');
    add_line(mdl, 'Sum/1', 'a/1');
    add_line(mdl, 'a/1', 'z^-1/1');
    add_line(mdl, 'z^-1/1', 'Sum/2');

    set_param(mdl, 'StopTime', '10');
    fprintf('\nSimulink model "%s" created successfully.\n', mdl);
catch ME
    fprintf('\nSimulink model creation skipped: %s\n', ME.message);
end
