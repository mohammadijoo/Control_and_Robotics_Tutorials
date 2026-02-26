% Chapter7_Lesson2.m
% UKF localization for planar (x, y, theta) robot with range-bearing landmarks.
%
% This script contains:
%   (1) A from-scratch additive-noise UKF for state x=[x;y;theta]
%   (2) A small simulation driven by differential-drive kinematics
%   (3) Optional: a reference snippet showing how to set up MATLAB's
%       unscentedKalmanFilter object (requires toolboxes; see comments)
%
% Note: This is written as a single script with local functions, which is
%       convenient for teaching and easy copy/paste into MATLAB.

clear; clc;

rng(7);

dt = 0.1;
T  = 250;

% landmarks
L = [ 5  0;
      0  6;
      6  6;
      8 -2 ];

% true state
x_true = [0; 0; 0.2];

% UKF parameters (common defaults)
params.alpha = 0.35;
params.beta  = 2.0;
params.kappa = 0.0;

% initial estimate
x = [0.5; -0.5; -0.3];
P = diag([0.8^2, 0.8^2, (20*pi/180)^2]);

% process noise
sigma_xy = 0.02;
sigma_th = 1.0*pi/180;
Q = diag([sigma_xy^2, sigma_xy^2, sigma_th^2]);

% measurement noise
sigma_r = 0.15;
sigma_b = 2.0*pi/180;
R = diag([sigma_r^2, sigma_b^2]);

xs_true = zeros(3, T);
xs_est  = zeros(3, T);

for k = 1:T
    v = 1.0 + 0.2*sin(0.07*(k-1));
    w = 0.35*sin(0.03*(k-1));
    u = [v; w];

    % propagate truth with noise
    x_true = f_motion(x_true, u, dt);
    x_true = x_true + [sigma_xy*randn; sigma_xy*randn; sigma_th*randn];
    x_true(3) = wrapAngle(x_true(3));

    % UKF prediction
    [x, P] = ukf_predict(x, P, Q, params, @(xx) f_motion(xx, u, dt));

    % sequential measurement updates (range-bearing to each landmark)
    for i = 1:size(L,1)
        lm = L(i,:)';
        z  = h_rangeBearing(x_true, lm) + [sigma_r*randn; sigma_b*randn];
        z(2) = wrapAngle(z(2));

        [x, P] = ukf_update(x, P, R, params, z, @(xx) h_rangeBearing(xx, lm));
    end

    xs_true(:,k) = x_true;
    xs_est(:,k)  = x;
end

pos_err = vecnorm(xs_est(1:2,:) - xs_true(1:2,:), 2, 1);
th_err  = abs(arrayfun(@(a,b) wrapAngle(a-b), xs_est(3,:), xs_true(3,:)));

fprintf('Final true state: [%.3f %.3f %.3f]\n', xs_true(:,end));
fprintf('Final UKF  state: [%.3f %.3f %.3f]\n', xs_est(:,end));
fprintf('RMSE position: %.4f\n', sqrt(mean(pos_err.^2)));
fprintf('RMSE theta   : %.4f\n', sqrt(mean(th_err.^2)));

% Optional plot
figure; hold on; grid on;
plot(xs_true(1,:), xs_true(2,:), 'LineWidth', 1.5);
plot(xs_est(1,:),  xs_est(2,:),  '--', 'LineWidth', 1.5);
plot(L(:,1), L(:,2), 'x', 'MarkerSize', 10, 'LineWidth', 1.5);
axis equal;
legend('true','ukf','landmarks');
title('UKF localization (range-bearing landmarks)');

%% Optional: MATLAB built-in UKF object (toolbox-dependent)
% The following snippet uses "unscentedKalmanFilter" (System Identification Toolbox)
% or related products depending on your installation.
%
% f = @(x,u) f_motion(x, u, dt);
% h = @(x)   h_rangeBearing(x, [5;0]); % one landmark as a demo
% ukfObj = unscentedKalmanFilter(@(x,u) f(x,u), h, x);
% ukfObj.ProcessNoise     = Q;
% ukfObj.MeasurementNoise = R;
% ukfObj.StateCovariance  = P;
% % Then loop:
% % xPred = predict(ukfObj, u);
% % xCorr = correct(ukfObj, z);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Local functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = f_motion(x, u, dt)
% Differential-drive kinematic model (pose state).
    px = x(1); py = x(2); th = x(3);
    v  = u(1); w  = u(2);
    y = [ px + dt*v*cos(th);
          py + dt*v*sin(th);
          wrapAngle(th + dt*w) ];
end

function z = h_rangeBearing(x, lm)
% Range-bearing measurement to a known landmark.
    px = x(1); py = x(2); th = x(3);
    dx = lm(1) - px;
    dy = lm(2) - py;
    r  = sqrt(dx^2 + dy^2);
    b  = wrapAngle(atan2(dy, dx) - th);
    z = [r; b];
end

function a = wrapAngle(a)
% Wrap angle to (-pi,pi].
    a = atan2(sin(a), cos(a));
end

function mu = meanAngle(angles, w)
% Weighted circular mean.
    s = sum(w .* sin(angles));
    c = sum(w .* cos(angles));
    mu = wrapAngle(atan2(s, c));
end

function [X, Wm, Wc] = sigmaPoints(x, P, params)
% Sigma point generation (additive-noise UKF).
    n = numel(x);
    alpha = params.alpha; beta = params.beta; kappa = params.kappa;
    lambda = alpha^2*(n+kappa) - n;

    S = chol((n+lambda)*P, 'lower');

    X = zeros(n, 2*n+1);
    X(:,1) = x;
    for i = 1:n
        X(:, 1+i)   = x + S(:,i);
        X(:, 1+i+n) = x - S(:,i);
    end
    X(3,:) = arrayfun(@wrapAngle, X(3,:));

    Wm = ones(2*n+1,1) * (1/(2*(n+lambda)));
    Wc = Wm;
    Wm(1) = lambda/(n+lambda);
    Wc(1) = Wm(1) + (1 - alpha^2 + beta);
end

function xbar = stateMean(X, Wm)
% Mean for pose state with angular component in index 3.
    xbar = zeros(3,1);
    xbar(1) = sum(Wm .* X(1,:)');
    xbar(2) = sum(Wm .* X(2,:)');
    xbar(3) = meanAngle(X(3,:)', Wm);
end

function dx = stateResidual(a, b)
% Residual a-b with wrapped angle on theta.
    dx = a - b;
    dx(3) = wrapAngle(dx(3));
end

function [xPred, PPred] = ukf_predict(x, P, Q, params, f)
% UKF time update: propagate sigma points through f(x).
    [X, Wm, Wc] = sigmaPoints(x, P, params);

    Xp = zeros(size(X));
    for i = 1:size(X,2)
        Xp(:,i) = f(X(:,i));
        Xp(3,i) = wrapAngle(Xp(3,i));
    end

    xPred = stateMean(Xp, Wm);

    PPred = zeros(3,3);
    for i = 1:size(Xp,2)
        dx = stateResidual(Xp(:,i), xPred);
        PPred = PPred + Wc(i) * (dx*dx');
    end
    PPred = PPred + Q;
end

function [xNew, PNew] = ukf_update(xPred, PPred, R, params, z, h)
% UKF measurement update for z=[range; bearing], bearing is angular.
    [X, Wm, Wc] = sigmaPoints(xPred, PPred, params);

    Z = zeros(2, size(X,2));
    for i = 1:size(X,2)
        zi = h(X(:,i));
        zi(2) = wrapAngle(zi(2));
        Z(:,i) = zi;
    end

    zbar = zeros(2,1);
    zbar(1) = sum(Wm .* Z(1,:)');
    zbar(2) = meanAngle(Z(2,:)', Wm);

    S = zeros(2,2);
    Pxz = zeros(3,2);
    for i = 1:size(X,2)
        dz = Z(:,i) - zbar;
        dz(2) = wrapAngle(dz(2));
        dx = stateResidual(X(:,i), xPred);
        S = S + Wc(i) * (dz*dz');
        Pxz = Pxz + Wc(i) * (dx*dz');
    end
    S = S + R;

    K = Pxz / S;
    innov = z - zbar;
    innov(2) = wrapAngle(innov(2));

    xNew = xPred + K*innov;
    xNew(3) = wrapAngle(xNew(3));

    PNew = PPred - K*S*K';
end
