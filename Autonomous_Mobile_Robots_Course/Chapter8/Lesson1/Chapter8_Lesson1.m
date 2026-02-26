% Chapter 8 - Particle-Filter Localization
% Lesson 1: Monte Carlo Localization (MCL) Intuition
%
% Minimal from-scratch MCL in a known 2D landmark map (range-only).
% State: x = [x; y; theta]
%
% Tested with base MATLAB. If you have Robotics System Toolbox, you can
% later compare against built-in localization utilities, but this script
% is fully stand-alone.

clear; clc; close all;
rng(0);

% Landmarks (square corners)
landmarks = [-5 -5;
              5 -5;
              5  5;
             -5  5];

% True pose
xTrue = [-3; -2; 0.3];

% Parameters
N = 800;
dt = 0.1;

sigmaV = 0.10;
sigmaW = 0.05;
sigmaR = 0.35;

% Initialize particles (broad prior)
P = zeros(3, N);
P(1,:) = 3.0 * randn(1,N);
P(2,:) = 3.0 * randn(1,N);
P(3,:) = -pi + 2*pi*rand(1,N);
w = ones(1,N) / N;

T = 120;
trajTrue = zeros(3, T+1);
trajEst  = zeros(3, T);
trajTrue(:,1) = xTrue;

for t = 1:T
    u = [0.6; 0.25]; % [v; omega]

    % True robot (noise-free for demo)
    xTrue = motionDeterministic(xTrue, u, dt);
    trajTrue(:,t+1) = xTrue;

    % Measurement: ranges with noise
    z = expectedRanges(xTrue, landmarks) + sigmaR*randn(size(landmarks,1),1);

    % MCL step
    [P, w, mu] = mclStep(P, w, u, z, landmarks, dt, sigmaV, sigmaW, sigmaR);
    trajEst(:,t) = mu;
end

figure; hold on; grid on; axis equal;
scatter(P(1,:), P(2,:), 8, 'filled', 'MarkerFaceAlpha', 0.15);
plot(trajTrue(1,:), trajTrue(2,:), 'LineWidth', 1.5);
plot(trajEst(1,:), trajEst(2,:), 'LineWidth', 1.5);
scatter(landmarks(:,1), landmarks(:,2), 80, 'x', 'LineWidth', 2);
legend('particles', 'true', 'estimate', 'landmarks');
title('Monte Carlo Localization (MCL) — Intuition Demo');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x = motionDeterministic(x, u, dt)
    v = u(1); w = u(2);
    th = x(3);
    if abs(w) < 1e-9
        x(1) = x(1) + v*dt*cos(th);
        x(2) = x(2) + v*dt*sin(th);
    else
        x(1) = x(1) + (v/w)*(sin(th+w*dt) - sin(th));
        x(2) = x(2) + (v/w)*(-cos(th+w*dt) + cos(th));
        x(3) = wrapAngle(th + w*dt);
    end
end

function x = motionSample(x, u, dt, sigmaV, sigmaW)
    v = u(1) + sigmaV*randn();
    w = u(2) + sigmaW*randn();
    th = x(3);
    if abs(w) < 1e-9
        x(1) = x(1) + v*dt*cos(th);
        x(2) = x(2) + v*dt*sin(th);
    else
        x(1) = x(1) + (v/w)*(sin(th+w*dt) - sin(th));
        x(2) = x(2) + (v/w)*(-cos(th+w*dt) + cos(th));
        x(3) = wrapAngle(th + w*dt);
    end
end

function r = expectedRanges(x, landmarks)
    dx = landmarks(:,1) - x(1);
    dy = landmarks(:,2) - x(2);
    r = sqrt(dx.^2 + dy.^2);
end

function lw = gaussianLogPdf(e, sigma)
    lw = -0.5*sum((e./sigma).^2) - numel(e)*log(sigma*sqrt(2*pi));
end

function idx = systematicResample(w)
    N = numel(w);
    cdf = cumsum(w);
    cdf(end) = 1.0;
    u0 = rand()/N;
    u = u0 + (0:(N-1))/N;
    idx = zeros(1,N);
    i = 1;
    for m = 1:N
        while u(m) > cdf(i) && i < N
            i = i + 1;
        end
        idx(m) = i;
    end
end

function a = wrapAngle(a)
    a = mod(a + pi, 2*pi);
    if a < 0, a = a + 2*pi; end
    a = a - pi;
end

function [P, w, mu] = mclStep(P, w, u, z, landmarks, dt, sigmaV, sigmaW, sigmaR)
    N = size(P,2);

    % Predict
    for i = 1:N
        P(:,i) = motionSample(P(:,i), u, dt, sigmaV, sigmaW);
    end

    % Update weights
    logw = zeros(1,N);
    for i = 1:N
        zhat = expectedRanges(P(:,i), landmarks);
        logw(i) = gaussianLogPdf(z - zhat, sigmaR);
    end
    logw = logw - max(logw);
    w = exp(logw);
    w = w / sum(w);

    % Resample
    idx = systematicResample(w);
    P = P(:,idx);
    w = ones(1,N)/N;

    % Mean pose (angle via circular mean)
    mu = mean(P,2);
    mu(3) = atan2(mean(sin(P(3,:))), mean(cos(P(3,:))));
end
