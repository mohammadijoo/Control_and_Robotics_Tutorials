% Chapter19_Lesson1.m
% Metrics for Localization and SLAM (Autonomous Mobile Robots)
% MATLAB reference implementation: SE(2) alignment, ATE, RPE, ANEES/NIS

clear; clc; rng(19);

N = 200;
tt = linspace(0,20,N)';
gt_xy = [0.5*tt, 2*sin(0.4*tt)];
dxy = [diff(gt_xy); gt_xy(end,:) - gt_xy(end-1,:)];
gt_yaw = atan2(dxy(:,2), dxy(:,1));

theta0 = deg2rad(7.0);
R0 = [cos(theta0) -sin(theta0); sin(theta0) cos(theta0)];
t0 = [1.5; -0.8];
drift = [0.01*tt, -0.004*tt];

est_xy = (R0' * (gt_xy' - t0))' + drift + 0.03*randn(N,2);
est_yaw = wrapToPi(gt_yaw - theta0 + 0.01*sin(0.2*tt) + 0.01*randn(N,1));

[R, t, theta] = alignSE2(gt_xy, est_xy);
est_xy_a = (R * est_xy')' + t';
est_yaw_a = wrapToPi(est_yaw + theta);

ate = sqrt(mean(sum((gt_xy - est_xy_a).^2, 2)));
[rpe_t, rpe_r] = rpeMetrics(gt_xy, gt_yaw, est_xy_a, est_yaw_a, 5);

% ANEES demo (state dimension n=3)
n = 3;
errs = [0.05*randn(N,1), 0.05*randn(N,1), deg2rad(1.0)*randn(N,1)];
Pk = repmat(diag([0.05^2, 0.05^2, deg2rad(1.1)^2]), 1, 1, N);
nees_vals = zeros(N,1);
for k = 1:N
    e = errs(k,:)';
    P = Pk(:,:,k);
    nees_vals(k) = e' * (P \ e);
end
anees_val = mean(nees_vals);

% NIS demo (measurement dimension m=2)
m = 2;
nu = [0.08*randn(N,1), deg2rad(2.0)*randn(N,1)];
Sk = repmat(diag([0.1^2, deg2rad(2.5)^2]), 1, 1, N);
nis_vals = zeros(N,1);
for k = 1:N
    v = nu(k,:)';
    S = Sk(:,:,k);
    nis_vals(k) = v' * (S \ v);
end
anis_val = mean(nis_vals);

fprintf('=== Chapter19 Lesson1 Metrics Demo (MATLAB) ===\n');
fprintf('ATE RMSE [m]: %.6f\n', ate);
fprintf('RPE translational RMSE [m] (delta=5): %.6f\n', rpe_t);
fprintf('RPE rotational RMSE [rad] (delta=5): %.6f\n', rpe_r);
fprintf('ANEES mean (n=3 expected near 3): %.6f\n', anees_val);
fprintf('NIS mean (m=2 expected near 2): %.6f\n', anis_val);
fprintf('Estimated global alignment theta [deg]: %.4f\n', rad2deg(theta));

function [R, t, theta] = alignSE2(gt_xy, est_xy)
    c_gt = mean(gt_xy,1);
    c_est = mean(est_xy,1);

    X = est_xy - c_est;
    Y = gt_xy - c_gt;

    s = sum(X(:,1).*Y(:,2) - X(:,2).*Y(:,1));
    c = sum(X(:,1).*Y(:,1) + X(:,2).*Y(:,2));
    theta = atan2(s, c);

    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    t = c_gt' - R * c_est';
end

function [rpe_t_rmse, rpe_r_rmse] = rpeMetrics(gt_xy, gt_yaw, est_xy, est_yaw, delta)
    M = size(gt_xy,1) - delta;
    et = zeros(M,1);
    er = zeros(M,1);
    for k = 1:M
        dg = gt_xy(k+delta,:) - gt_xy(k,:);
        de = est_xy(k+delta,:) - est_xy(k,:);
        et(k) = norm(dg - de, 2);

        dyg = wrapToPi(gt_yaw(k+delta) - gt_yaw(k));
        dye = wrapToPi(est_yaw(k+delta) - est_yaw(k));
        er(k) = abs(wrapToPi(dyg - dye));
    end
    rpe_t_rmse = sqrt(mean(et.^2));
    rpe_r_rmse = sqrt(mean(er.^2));
end

function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end
