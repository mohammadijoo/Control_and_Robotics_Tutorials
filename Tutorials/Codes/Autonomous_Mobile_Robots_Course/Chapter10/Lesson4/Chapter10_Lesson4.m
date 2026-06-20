% Chapter10_Lesson4.m
% Robust scan matching (2D) under dynamic obstacles using IRLS-Huber + trimming
% This script is self-contained and uses only base MATLAB.
%
% Simulink note:
%   You can wrap robust_icp_2d below inside a MATLAB Function block (or call it
%   from a MATLAB System block) to integrate with a larger estimation pipeline.

function Chapter10_Lesson4()
    rng(7);

    [scan1, scan2, R_true, t_true] = make_synthetic_scene(250, 60);

    iters = 25;
    delta = 0.18;
    keep_ratio = 0.85;

    [R_est, t_est, rmse_hist] = robust_icp_2d(scan1, scan2, iters, delta, keep_ratio);

    [ang_err_deg, trans_err] = pose_error(R_est, t_est, R_true, t_true);

    fprintf('True t: [%.3f %.3f]\\n', t_true(1), t_true(2));
    fprintf('Est  t: [%.3f %.3f]\\n', t_est(1), t_est(2));
    fprintf('Angle error (deg): %.3f\\n', ang_err_deg);
    fprintf('Translation error : %.3f\\n', trans_err);
    fprintf('RMSE (first 5): %s ... last: %.4f\\n', mat2str(rmse_hist(1:min(5,end))), rmse_hist(end));
end

function R = rot2(theta)
    c = cos(theta); s = sin(theta);
    R = [c -s; s c];
end

function [R, t] = weighted_kabsch_2d(P, Q, w)
    w = w(:);
    wsum = sum(w) + 1e-12;
    pbar = (w' * P) / wsum;
    qbar = (w' * Q) / wsum;

    X = P - pbar;
    Y = Q - qbar;

    S = zeros(2,2);
    for i=1:size(P,1)
        S = S + w(i) * (X(i,:)' * Y(i,:));
    end

    num = S(1,2) - S(2,1);
    den = S(1,1) + S(2,2);
    theta = atan2(num, den);

    R = rot2(theta);
    t = qbar' - R*pbar';
end

function [idx, d2] = nearest_neighbors_bruteforce(A, B)
    N = size(A,1);
    idx = zeros(N,1);
    d2  = zeros(N,1);
    for i=1:N
        dif = B - A(i,:);
        dist2 = sum(dif.^2, 2);
        [d2(i), idx(i)] = min(dist2);
    end
end

function w = huber_weights(r, delta)
    w = ones(size(r));
    mask = r > delta;
    w(mask) = delta ./ (r(mask) + 1e-12);
end

function [R, t, rmse_hist] = robust_icp_2d(source, target, iters, delta, keep_ratio)
    src = source;
    tgt = target;

    R = eye(2);
    t = [0; 0];
    rmse_hist = zeros(iters,1);

    for k=1:iters
        src_w = (R*src')' + t';  % Nx2

        [nn_idx, d2] = nearest_neighbors_bruteforce(src_w, tgt);
        Q = tgt(nn_idx, :);
        res = src_w - Q;
        r = sqrt(sum(res.^2, 2));

        thr = quantile(r, keep_ratio);
        inliers = r <= thr;

        P_in = src(inliers, :);
        Q_in = Q(inliers, :);
        r_in = r(inliers);

        w = huber_weights(r_in, delta);

        [dR, dt] = weighted_kabsch_2d(P_in, Q_in, w);

        R = dR * R;
        t = dR * t + dt;

        rmse_hist(k) = sqrt(mean(r_in.^2));
    end
end

function [scan1, scan2, R_true, t_true] = make_synthetic_scene(n_static, n_dyn)
    ang = 2*pi*rand(n_static/2,1);
    circle = [2*cos(ang), 2*sin(ang)];
    line1  = [-3 + 6*rand(n_static/4,1), -1.5*ones(n_static/4,1)];
    line2  = [1.5*ones(n_static/4,1), -2 + 4*rand(n_static/4,1)];
    stat = [circle; line1; line2];

    dyn_c1 = [-0.5, 0.8];
    dyn1 = dyn_c1 + 0.15*randn(n_dyn,2);

    theta_true = deg2rad(12.0);
    t_true = [0.35; -0.10];
    R_true = rot2(theta_true);

    scan1 = [stat; dyn1] + 0.02*randn(size(stat,1)+n_dyn, 2);

    dyn_c2 = dyn_c1 + [0.55, -0.25];
    dyn2 = dyn_c2 + 0.15*randn(n_dyn,2);

    stat2 = (R_true*stat')' + t_true';
    scan2 = [stat2; dyn2] + 0.02*randn(size(stat2,1)+n_dyn, 2);
end

function [ang_err_deg, trans_err] = pose_error(R_est, t_est, R_true, t_true)
    dR = R_est * R_true';
    ang = atan2(dR(2,1), dR(1,1));
    ang_err_deg = abs(rad2deg(ang));
    trans_err = norm(t_est - t_true);
end
