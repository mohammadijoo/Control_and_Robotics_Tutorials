% Chapter10_Lesson5.m
% Autonomous Mobile Robots — Chapter 10, Lesson 5
% Lab: ICP-Based Motion Estimation (2D)
%
% This script implements a basic 2D point-to-point ICP to estimate relative
% motion between two LiDAR scans (2D point sets).
%
% MATLAB toolboxes (optional):
% - Robotics System Toolbox (for pointCloud, pcregistericp in 3D workflows)
% - ROS Toolbox (for reading LaserScan messages)
%
% Run:
%   Chapter10_Lesson5

function Chapter10_Lesson5()

rng(7);

% ----- Simulate world points (L-corner + scatter) -----
n = 700;
world = [8*rand(n/4,1), zeros(n/4,1); ...
         zeros(n/4,1), 6*rand(n/4,1); ...
         8*rand(n/2,1), 6*rand(n/2,1)];

% Pose k (identity)
R0 = rot2(0);
t0 = [0;0];

% Pose k+1 (ground truth motion)
thetaGT = deg2rad(6);
R1 = rot2(thetaGT);
t1 = [0.35; 0.10];

scanK  = scanFromPose(world, R0, t0, 0.01, 1);
scanK1 = scanFromPose(world, R1, t1, 0.01, 2);

% Weak initial guess (odometry-like)
Rinit = rot2(deg2rad(3));
tinit = [0.20; 0.00];

% ----- Run ICP: estimate transform mapping scan_{k+1} -> scan_k -----
opts.maxIter = 60;
opts.tol = 1e-7;
opts.rejectDist = 0.5;
opts.trimFraction = 0.85;

[Rhat, that, rmseHist] = icp2d(scanK1, scanK, Rinit, tinit, opts);

thetaHat = atan2(Rhat(2,1), Rhat(1,1));

fprintf("=== ICP-Based Motion Estimation (2D) ===\n");
fprintf("Ground truth: theta = %.3f deg, t = [%.3f, %.3f]\n", rad2deg(thetaGT), t1(1), t1(2));
fprintf("Estimated   : theta = %.3f deg, t = [%.3f, %.3f]\n", rad2deg(thetaHat), that(1), that(2));
fprintf("Iterations  : %d, final RMSE ≈ %.6f\n", numel(rmseHist), rmseHist(end));

% ----- Simulink note -----
% To use inside Simulink:
% 1) Create a MATLAB Function block.
% 2) Put a persistent previousScan and previousPose in the block.
% 3) Call a helper like icp2d() to compute [Rhat,that] between scans.
% 4) Convert [Rhat,that] to delta pose and integrate.
%
% Keep the ICP iteration budget small (e.g., 10-20) for real-time execution.

end

% ---------------- Helpers ----------------

function R = rot2(theta)
c = cos(theta); s = sin(theta);
R = [c -s; s c];
end

function scan = scanFromPose(world, R_wb, t_wb, noiseStd, seed)
rng(seed);
R_bw = R_wb';
P = (R_bw * (world' - t_wb)).';  % body-frame points

r = sqrt(sum(P.^2,2));
ang = atan2(P(:,2), P(:,1));
maxRange = 10.0;
fov = deg2rad(270);

mask = (r <= maxRange) & (abs(ang) <= fov/2);
scan = P(mask,:) + noiseStd*randn(sum(mask),2);

% add outliers
kout = max(5, floor(size(scan,1)/40));
out = -2 + 4*rand(kout,2);
scan = [scan; out];
end

function [R, t, rmseHist] = icp2d(src0, dst0, R0, t0, opts)
R = R0; t = t0(:);
rmseHist = [];
prev = inf;

for it = 1:opts.maxIter
    srcT = (R*src0.' + t).';  % transform src

    [nn, dist] = nearestNeighbors(srcT, dst0);

    % gating
    mask = dist <= opts.rejectDist;

    % trimming
    dist2 = dist(mask);
    srcM = srcT(mask,:);
    nnM  = nn(mask,:);
    if size(srcM,1) < 3
        error("Too few correspondences after filtering.");
    end

    if opts.trimFraction < 1
        [~, idx] = sort(dist2, 'ascend');
        k = max(3, floor(opts.trimFraction * numel(idx)));
        idx = idx(1:k);
        srcM = srcM(idx,:);
        nnM  = nnM(idx,:);
        dist2 = dist2(idx);
    end

    [dR, dt] = bestFit2D(srcM, nnM);
    R = dR * R;
    t = dR * t + dt;

    rmse = sqrt(mean(dist2.^2));
    rmseHist(end+1) = rmse; %#ok<AGROW>
    if abs(prev - rmse) < opts.tol
        break;
    end
    prev = rmse;
end

end

function [nn, dist] = nearestNeighbors(P, Q)
% brute-force NN (for clarity)
N = size(P,1);
nn = zeros(N,2);
dist = zeros(N,1);
for i = 1:N
    dif = Q - P(i,:);
    d2 = sum(dif.^2,2);
    [m, j] = min(d2);
    nn(i,:) = Q(j,:);
    dist(i) = sqrt(m);
end
end

function [R, t] = bestFit2D(src, dst)
% Closed-form 2D least-squares rotation without SVD.
muS = mean(src,1).';
muD = mean(dst,1).';

Xs = src.' - muS;
Yd = dst.' - muD;

A = sum(Xs(1,:).*Yd(1,:) + Xs(2,:).*Yd(2,:));
B = sum(Xs(1,:).*Yd(2,:) - Xs(2,:).*Yd(1,:));
theta = atan2(B, A);
R = rot2(theta);
t = muD - R*muS;
end
