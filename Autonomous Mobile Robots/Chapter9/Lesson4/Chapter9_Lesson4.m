% Chapter9_Lesson4.m
% Elevation + traversability maps (outdoor AMR) — MATLAB + Simulink-friendly script.
% This script:
%   1) Generates a synthetic point cloud for terrain
%   2) Fuses points into a probabilistic elevation grid (per-cell 1D Kalman)
%   3) Computes slope/roughness/step and a traversability cost map
%   4) Optionally builds a simple Simulink model for the per-cell Kalman update

clear; clc; close all;

% -----------------------------
% Parameters
% -----------------------------
xMin = -10; xMax = 10;
yMin = -10; yMax = 10;
res = 0.20;

nx = ceil((xMax - xMin)/res);
ny = ceil((yMax - yMin)/res);

sigma0 = 2.0;       % initial std (m)
measSigma = 0.10;   % sensor std (m)
R = measSigma^2;
gateK = 3.0;

mu = zeros(nx, ny);
sigma2 = (sigma0^2) * ones(nx, ny);
count = zeros(nx, ny);

% -----------------------------
% Synthetic point cloud
% -----------------------------
rng(7);
nGround = 5e4;
nOut = 2e3;

x = (xMin + (xMax-xMin)*rand(nGround,1));
y = (yMin + (yMax-yMin)*rand(nGround,1));

zTrue = 0.20*sin(0.35*x) + 0.15*cos(0.25*y) + 0.03*x ...
        + 0.25*exp(-0.08*((x-2).^2 + (y+1).^2));

zMeas = zTrue + measSigma*randn(nGround,1);
ground = [x, y, zMeas];

xo = (xMin + (xMax-xMin)*rand(nOut,1));
yo = (yMin + (yMax-yMin)*rand(nOut,1));
zo = 0.20*sin(0.35*xo) + 0.15*cos(0.25*yo) + 0.03*xo ...
     + 0.25*exp(-0.08*((xo-2).^2 + (yo+1).^2)) ...
     + 0.8 + 0.05*randn(nOut,1);
outl = [xo, yo, zo];

pts = [ground; outl];
pts = pts(randperm(size(pts,1)), :);

% -----------------------------
% Per-cell Kalman fusion
% -----------------------------
accepted = 0;
for k = 1:size(pts,1)
    px = pts(k,1); py = pts(k,2); pz = pts(k,3);

    ix = floor((px - xMin)/res) + 1;
    iy = floor((py - yMin)/res) + 1;

    if ix < 1 || ix > nx || iy < 1 || iy > ny
        continue;
    end

    m = mu(ix,iy);
    s2 = sigma2(ix,iy);

    nu = pz - m;
    S = s2 + R;

    if count(ix,iy) > 0 && abs(nu) > gateK*sqrt(S)
        continue; % reject outlier/obstacle
    end

    K = s2 / S;
    mu(ix,iy) = m + K*nu;
    sigma2(ix,iy) = (1 - K)*s2;
    count(ix,iy) = count(ix,iy) + 1;
    accepted = accepted + 1;
end

fprintf('Points accepted: %d out of %d\n', accepted, size(pts,1));

% -----------------------------
% Features: slope, roughness, step
% -----------------------------
slope = zeros(nx,ny);
rough = zeros(nx,ny);
stepH = zeros(nx,ny);

for i = 2:nx-1
    for j = 2:ny-1
        if count(i,j) == 0
            continue;
        end

        dzdx = (mu(i+1,j) - mu(i-1,j)) / (2*res);
        dzdy = (mu(i,j+1) - mu(i,j-1)) / (2*res);
        slope(i,j) = atan(sqrt(dzdx^2 + dzdy^2));

        neigh = mu(i-1:i+1, j-1:j+1);
        rough(i,j) = std(neigh(:));

        stepH(i,j) = max(abs(neigh(:) - mu(i,j)));
    end
end

% -----------------------------
% Traversability cost
% -----------------------------
slopeRef = 0.35;  % rad
roughRef = 0.08;  % m
stepRef  = 0.12;  % m

wS = 3.0; wR = 2.0; wD = 2.5; bias = -3.0;

s = slope / max(1e-9, slopeRef);
r = rough / max(1e-9, roughRef);
d = stepH / max(1e-9, stepRef);
score = wS*s + wR*r + wD*d + bias;

cost = 1 ./ (1 + exp(-score));

% -----------------------------
% Plots
% -----------------------------
figure; imagesc(mu'); axis image; colorbar;
title('Elevation mean (m)'); set(gca,'YDir','normal');

figure; imagesc(cost'); axis image; colorbar;
title('Traversability cost (0 easy, 1 hard)'); set(gca,'YDir','normal');

% -----------------------------
% Export
% -----------------------------
writematrix(mu, 'Chapter9_Lesson4_elevation_mu.csv');
writematrix(cost, 'Chapter9_Lesson4_traversability_cost.csv');
writematrix(count, 'Chapter9_Lesson4_fusion_count.csv');

% -----------------------------
% Optional: build a Simulink model that implements ONE cell's 1D Kalman update.
% The idea is to show how the recursion (mu, sigma2) can be implemented as discrete blocks.
% Run: buildSimulinkKalmanCell();
% -----------------------------
% buildSimulinkKalmanCell();

function buildSimulinkKalmanCell()
    model = 'Chapter9_Lesson4_KalmanCell';
    if bdIsLoaded(model); close_system(model, 0); end
    new_system(model); open_system(model);

    add_block('simulink/Sources/In1', [model '/z_meas'], 'Position', [30 40 60 60]);
    add_block('simulink/Sources/In1', [model '/R'], 'Position', [30 90 60 110]);
    add_block('simulink/Discrete/Unit Delay', [model '/mu(z^-1)'], 'Position', [140 30 200 70]);
    add_block('simulink/Discrete/Unit Delay', [model '/sigma2(z^-1)'], 'Position', [140 90 200 130]);

    add_block('simulink/User-Defined Functions/MATLAB Function', [model '/KalmanUpdate'], ...
              'Position', [260 40 420 120]);
    set_param([model '/KalmanUpdate'], 'Script', ...
        ['function [mu_p, sigma2_p] = f(mu, sigma2, z, R)\n' ...
         '% 1D Kalman update: K = sigma2 / (sigma2 + R)\n' ...
         'S = sigma2 + R;\n' ...
         'K = sigma2 / S;\n' ...
         'mu_p = mu + K*(z - mu);\n' ...
         'sigma2_p = (1 - K)*sigma2;\n' ...
         'end\n']);

    add_block('simulink/Sinks/Out1', [model '/mu_plus'], 'Position', [470 45 500 65]);
    add_block('simulink/Sinks/Out1', [model '/sigma2_plus'], 'Position', [470 95 500 115]);

    add_line(model, 'z_meas/1', 'KalmanUpdate/3');
    add_line(model, 'R/1', 'KalmanUpdate/4');
    add_line(model, 'mu(z^-1)/1', 'KalmanUpdate/1');
    add_line(model, 'sigma2(z^-1)/1', 'KalmanUpdate/2');

    add_line(model, 'KalmanUpdate/1', 'mu_plus/1');
    add_line(model, 'KalmanUpdate/2', 'sigma2_plus/1');

    add_line(model, 'KalmanUpdate/1', 'mu(z^-1)/1', 'autorouting', 'on');
    add_line(model, 'KalmanUpdate/2', 'sigma2(z^-1)/1', 'autorouting', 'on');

    save_system(model);
    disp(['Saved Simulink model: ' model '.slx']);
end
