% Chapter6_Lesson5.m
% Lab: Build a Basic Bayes Filter Loop (discrete 1D cyclic state)
%
% This script runs a simple Bayes filter on a 1D cyclic track with:
%   - motion model: cyclic Gaussian translation kernel
%   - sensor model: signed displacement to a landmark + Gaussian noise
%
% Requirements:
%   - MATLAB (no special toolboxes required for the core script)
% Optional:
%   - Robotics System Toolbox for later ROS integration (not required here)

clear; clc;

% Discretization
L = 10.0;          % [m]
N = 200;
dx = L / N;
halfPeriod = L / 2;

% Models
uDeltaM = 0.35;    % [m] commanded translation per step
sigmaM  = 0.12;    % [m]
sigmaZ  = 0.20;    % [m]
landmarkX = 2.0;   % [m]

% Kernel (cyclic offsets in [-N/2, N/2))
muCells = uDeltaM / dx;
sigmaCells = sigmaM / dx;

idx = 0:(N-1);
d = idx;
half = floor(N/2);
d(d >= half) = d(d >= half) - N;     % negative offsets
k = exp(-0.5 * ((d - muCells) / sigmaCells).^2);
k = k / sum(k);

% Initial belief: uniform
bel = ones(N,1) / N;

% Ground truth (simulation only)
rng(7);
xTrue = 7.0;

T = 25;
fprintf("t, x_true[m], x_hat_MAP[m], z_meas[m]\n");

for t = 1:T
    % Motion simulation
    xTrue = mod(xTrue + uDeltaM + sigmaM * randn(), L);

    % Sensor simulation: signed displacement in [-L/2, L/2)
    zTrue = wrapInterval(xTrue - landmarkX, halfPeriod);
    zMeas = wrapInterval(zTrue + sigmaZ * randn(), halfPeriod);

    % Prediction: cyclic convolution via FFT
    belBar = real(ifft(fft(bel) .* fft(k(:))));
    belBar = max(belBar, 0);
    belBar = belBar / sum(belBar);

    % Likelihood
    xs = (0:(N-1))' * dx;
    dispToLm = arrayfun(@(x) wrapInterval(x - landmarkX, halfPeriod), xs);
    l = normpdf(zMeas, dispToLm, sigmaZ);
    l = max(l, 1e-300);

    % Update
    bel = belBar .* l;
    bel = bel / sum(bel);

    % MAP estimate
    [~, idxHat] = max(bel);
    xHat = (idxHat - 1) * dx;

    fprintf("%2d, %7.3f, %10.3f, %7.3f\n", t, xTrue, xHat, zMeas);
end

% Posterior entropy (sanity check)
eps = 1e-12;
H = -sum(bel .* log(bel + eps));
fprintf("Posterior entropy (nats): %.4f\n", H);

% --- Local function(s)
function y = wrapInterval(x, halfPeriod)
    % Wrap x into [-halfPeriod, +halfPeriod)
    period = 2.0 * halfPeriod;
    y = mod(x + halfPeriod, period) - halfPeriod;
end
