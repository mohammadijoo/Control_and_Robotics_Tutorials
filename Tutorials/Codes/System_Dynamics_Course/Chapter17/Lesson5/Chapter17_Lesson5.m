% Chapter17_Lesson5.m
% Monte Carlo simulation of a stochastic mass-spring-damper system
% m*xdd + c*xd + k*x = F(t)
% Randomness in k, c, initial conditions, and sampled forcing.
clear; clc; close all;

rng(17);

T = 10.0;
dt = 0.005;
N = 2000;
m = 1.0;

kMean = 25.0; kStd = 2.0;
cMean = 1.5;  cStd = 0.2;
x0Mean = 0.0; x0Std = 0.05;
v0Mean = 0.0; v0Std = 0.05;
sigmaF = 2.0;
xThreshold = 0.75;

t = 0:dt:T;
nt = numel(t);

sumX = zeros(1, nt);
sumX2 = zeros(1, nt);
exceedCount = 0;

for trial = 1:N
    k = max(1e-6, kMean + kStd * randn());
    c = max(1e-6, cMean + cStd * randn());

    x = x0Mean + x0Std * randn();
    v = v0Mean + v0Std * randn();

    xHist = zeros(1, nt);
    xHist(1) = x;

    peakAbs = abs(x);

    for n = 1:(nt - 1)
        F = sigmaF * randn();
        [x, v] = rk4Step(x, v, F, k, c, m, dt);
        xHist(n + 1) = x;
        peakAbs = max(peakAbs, abs(x));
    end

    sumX = sumX + xHist;
    sumX2 = sumX2 + xHist.^2;

    if peakAbs > xThreshold
        exceedCount = exceedCount + 1;
    end
end

meanX = sumX / N;
varX = (sumX2 - N * meanX.^2) / (N - 1);
varX = max(varX, 0);
stdX = sqrt(varX);

z975 = 1.959963984540054;
seFinal = stdX(end) / sqrt(N);
ciMean = [meanX(end) - z975 * seFinal, meanX(end) + z975 * seFinal];

pHat = exceedCount / N;
seP = sqrt(max(pHat * (1 - pHat), 1e-12) / N);
ciP = [max(0, pHat - z975 * seP), min(1, pHat + z975 * seP)];

fprintf('Monte Carlo trajectories: %d\n', N);
fprintf('Final mean x(T): %.6f\n', meanX(end));
fprintf('Final variance x(T): %.6f\n', varX(end));
fprintf('95%% CI for E[x(T)]: [%.6f, %.6f]\n', ciMean(1), ciMean(2));
fprintf('P(max |x| > %.3f): %.6f\n', xThreshold, pHat);
fprintf('95%% CI for probability: [%.6f, %.6f]\n', ciP(1), ciP(2));

resultTable = table(t(:), meanX(:), varX(:), stdX(:), ...
    'VariableNames', {'t','mean_x','var_x','std_x'});
writetable(resultTable, 'Chapter17_Lesson5_matlab_results.csv');

figure;
plot(t, meanX, 'LineWidth', 1.5); hold on;
plot(t, meanX + 2 * stdX, '--', 'LineWidth', 1.0);
plot(t, meanX - 2 * stdX, '--', 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Displacement x(t)');
title('Monte Carlo response statistics');
legend('Mean', 'Mean + 2 std', 'Mean - 2 std', 'Location', 'best');

% ---------------- Simulink note ----------------
% Equivalent Simulink model:
% 1) State-Space block or two Integrator blocks for x and v
% 2) Random Number block (or Band-Limited White Noise) for F(t)
% 3) MATLAB Function block to sample random k,c per run (or use parsim)
% 4) To Workspace blocks to collect x(t)
% 5) Monte Carlo wrapper script using repeated sim(...) calls

function [xNext, vNext] = rk4Step(x, v, force, k, c, m, h)
    f = @(state) [state(2);
                  -(k / m) * state(1) - (c / m) * state(2) + force / m];

    y = [x; v];
    k1 = f(y);
    k2 = f(y + 0.5 * h * k1);
    k3 = f(y + 0.5 * h * k2);
    k4 = f(y + h * k3);

    yNext = y + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
    xNext = yNext(1);
    vNext = yNext(2);
end
