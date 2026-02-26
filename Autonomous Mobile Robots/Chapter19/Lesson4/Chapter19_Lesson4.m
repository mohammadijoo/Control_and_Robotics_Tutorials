% Chapter19_Lesson4.m
% AMR stress testing + paired ablation (MATLAB / Simulink support script)

clear; clc; rng(42);

stressGrid = [0.0 0.2 0.4 0.6 0.8];
seeds = 100:149;

modsFull = [1 1 1];      % [scan imu predict]
modsNoScan = [0 1 1];
modsNoIMU = [1 0 1];

fprintf('stress   pSucc(full)   score(full)   pairedDelta(NoScan)   pairedDelta(NoIMU)\n');

for s = stressGrid
    fullScores = zeros(size(seeds));
    noScanScores = zeros(size(seeds));
    noIMUScores = zeros(size(seeds));
    fullSucc = zeros(size(seeds));

    for k = 1:numel(seeds)
        sd = seeds(k);
        eFull = simulateEpisode(sd, s, modsFull);
        eNoScan = simulateEpisode(sd, s, modsNoScan); % paired seed
        eNoIMU = simulateEpisode(sd, s, modsNoIMU);   % paired seed

        fullScores(k) = missionScore(eFull);
        noScanScores(k) = missionScore(eNoScan);
        noIMUScores(k) = missionScore(eNoIMU);
        fullSucc(k) = eFull.success;
    end

    fprintf('%4.1f      %7.3f       %8.2f          %8.2f            %8.2f\n', ...
        s, mean(fullSucc), mean(fullScores), mean(fullScores - noScanScores), mean(fullScores - noIMUScores));
end

% Reliability curve plot
pSucc = zeros(size(stressGrid));
for i = 1:numel(stressGrid)
    s = stressGrid(i);
    tmp = zeros(size(seeds));
    for k = 1:numel(seeds)
        tmp(k) = simulateEpisode(seeds(k), s, modsFull).success;
    end
    pSucc(i) = mean(tmp);
end
figure; plot(stressGrid, pSucc, '-o', 'LineWidth', 1.5); grid on;
xlabel('Stress level'); ylabel('Success probability');
title('AMR reliability curve under stress');

function ep = simulateEpisode(seed, stress, mods)
    rng(seed, 'twister');
    dyn = poissrnd(2 + 6*stress);
    dropout = rand < min(0.85, 0.05 + 0.75*stress);
    slip = abs(normrnd(0, 0.02 + 0.12*stress));
    bias = abs(normrnd(0, 0.01 + 0.08*stress));
    missing = 0.9*(1-mods(1)) + 0.8*(1-mods(2)) + 0.7*(1-mods(3));

    rmse = 0.05 + 0.09*stress + 0.04*dyn + 1.4*slip + 1.6*bias + 0.10*dropout + 0.12*missing + normrnd(0,0.02);
    rmse = max(0.01, rmse);
    margin = 0.55 - 0.18*stress - 0.01*dyn - 0.07*dropout - 0.05*missing + 0.02*mods(3) + normrnd(0,0.03);
    pathRatio = 1.03 + 0.14*stress + 0.02*dyn + 0.08*dropout + 0.05*missing - 0.03*mods(1) + normrnd(0,0.02);
    pathRatio = max(1.0, pathRatio);
    ttc = max(5.0, 34 + 7*pathRatio + 2.0*dyn + 8.0*stress + 3.5*missing + normrnd(0,1.5));

    logitVal = 3.2 - 2.6*stress - 2.2*rmse + 1.0*margin - 0.5*(pathRatio - 1) - 0.7*missing;
    p = 1/(1 + exp(-logitVal));
    ep = struct('success', rand < p, 'rmse', rmse, 'margin', margin, 'pathRatio', pathRatio, 'ttc', ttc);
end

function y = missionScore(ep)
    y = 100 - 35*(1-ep.success) ...
          - 16*max(0, ep.rmse - 0.10) ...
          - 14*max(0, 0.25 - ep.margin) ...
          - 5*max(0, ep.pathRatio - 1.10) ...
          - 0.20*max(0, ep.ttc - 45.0);
    y = max(0, y);
end
