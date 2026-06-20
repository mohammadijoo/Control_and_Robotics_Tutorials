% Chapter1_Lesson4.m
% Sensing–Estimation–Navigation pipeline demo (2D unicycle) using WLS (prior + range-bearing).
% Requires: base MATLAB. Optional (not required): Robotics System Toolbox / Simulink for later chapters.
% Run:
%   Chapter1_Lesson4

clear; clc; close all;
rng(7);

wrapAngle = @(a) mod(a + pi, 2*pi) - pi;

unicycleStep = @(x,v,w,dt) [ ...
    x(1) + v*dt*cos(x(3)); ...
    x(2) + v*dt*sin(x(3)); ...
    wrapAngle(x(3) + w*dt) ];

measModel = @(x,lm) [ ...
    hypot(lm(1)-x(1), lm(2)-x(2)); ...
    wrapAngle(atan2(lm(2)-x(2), lm(1)-x(1)) - x(3)) ];

measJacobian = @(x,lm) localMeasJacobian(x,lm);

% Map (known landmarks)
landmarks = [5 0; 6 6; 0 6];

dt = 0.1; N = 350;

xTrue = [0;0;0];
xHat  = [0;0;0];

P = diag([0.15^2, 0.15^2, deg2rad(8)^2]);

sigmaV = 0.08;
sigmaW = deg2rad(3);
sigmaR = 0.12;
sigmaB = deg2rad(2);

goal = [7;7];
kHeading = 1.6;
vMax = 0.9;

R = diag([sigmaR^2, sigmaB^2]);

histTrue = zeros(N,3);
histHat  = zeros(N,3);

for k=1:N
    % --- Navigation (policy uses estimate) ---
    dx = goal(1) - xHat(1);
    dy = goal(2) - xHat(2);
    dist = hypot(dx,dy);
    desired = atan2(dy,dx);
    headingErr = wrapAngle(desired - xHat(3));
    vCmd = vMax*tanh(dist);
    wCmd = kHeading*headingErr;

    % --- True motion with mild slip disturbance ---
    slip = deg2rad(0.35)*randn;
    xTrue = unicycleStep(xTrue, vCmd, wCmd + slip/dt, dt);

    % --- Odometry ---
    vMeas = vCmd + sigmaV*randn;
    wMeas = wCmd + sigmaW*randn;

    % --- Prediction ---
    xPrior = unicycleStep(xHat, vMeas, wMeas, dt);

    th = xHat(3);
    F = [1 0 -vMeas*dt*sin(th);
         0 1  vMeas*dt*cos(th);
         0 0  1];

    Q = diag([(sigmaV*dt)^2, (sigmaV*dt)^2, (sigmaW*dt)^2]);
    PPrior = F*P*F' + Q;

    % --- Landmark measurements ---
    zList = [];
    lmList = [];
    maxRange = 8.0;
    for j=1:size(landmarks,1)
        zTrue = measModel(xTrue, landmarks(j,:)');
        if zTrue(1) <= maxRange
            zNoisy = zTrue + [sigmaR*randn; sigmaB*randn];
            zNoisy(2) = wrapAngle(zNoisy(2));
            zList = [zList; zNoisy];
            lmList = [lmList; landmarks(j,:)];
        end
    end

    % --- Iterated WLS correction (2 iterations) ---
    [xHat, P] = wlsUpdateIterated(xPrior, PPrior, zList, lmList, R, 2, wrapAngle, measModel, measJacobian);

    histTrue(k,:) = xTrue';
    histHat(k,:)  = xHat';
end

figure;
plot(histTrue(:,1), histTrue(:,2), 'LineWidth', 1.5); hold on;
plot(histHat(:,1), histHat(:,2),  'LineWidth', 1.5);
scatter(landmarks(:,1), landmarks(:,2), 70, 'x', 'LineWidth', 2);
scatter(goal(1), goal(2), 90, '*', 'LineWidth', 2);
axis equal; grid on;
legend('true','estimated','landmarks','goal');
title('Sensing–Estimation–Navigation loop (demo)');

% ===== Local functions =====

function H = localMeasJacobian(x,lm)
    dx = lm(1) - x(1);
    dy = lm(2) - x(2);
    q = dx^2 + dy^2;
    q = max(q, 1e-12);
    r = sqrt(q);
    r = max(r, 1e-12);

    H = zeros(2,3);
    H(1,1) = -dx/r;
    H(1,2) = -dy/r;
    H(1,3) = 0;

    H(2,1) =  dy/q;
    H(2,2) = -dx/q;
    H(2,3) = -1;
end

function [xPost, PPost] = wlsUpdateIterated(xPrior, PPrior, zList, lmList, R, iters, wrapAngle, measModel, measJacobian)
    if isempty(zList)
        xPost = xPrior;
        PPost = PPrior;
        return;
    end

    m = size(zList,1);
    z = reshape(zList', [], 1); % (2m x 1)

    Rbig = kron(eye(m), R);
    Rinv = inv(Rbig);
    Pinv = inv(PPrior);

    x = xPrior;
    for it = 1:iters
        h = zeros(2*m,1);
        H = zeros(2*m,3);
        for j=1:m
            lm = lmList(j,:)';
            hj = measModel(x,lm);
            h(2*j-1:2*j) = hj;
            H(2*j-1:2*j,:) = measJacobian(x,lm);
        end

        res = z - h;
        for j=1:m
            res(2*j) = wrapAngle(res(2*j));
        end

        A = Pinv + H'*Rinv*H;
        b = H'*Rinv*res + Pinv*(xPrior - x);

        delta = A \ b;
        x = x + delta;
        x(3) = wrapAngle(x(3));
    end

    % Posterior covariance at final linearization point
    H = zeros(2*m,3);
    for j=1:m
        lm = lmList(j,:)';
        H(2*j-1:2*j,:) = measJacobian(x,lm);
    end
    PPost = inv(Pinv + H'*Rinv*H);
    xPost = x;
end
