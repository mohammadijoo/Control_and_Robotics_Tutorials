% Chapter19_Lesson3.m
% Standard Datasets and Simulated Benchmarks for AMR
% MATLAB/Simulink-oriented script for trajectory benchmark metrics (SE(2)).
%
% CSV columns required: t, x, y, yaw

clear; clc;

gtFile  = 'gt_traj.csv';
estFile = 'est_traj.csv';

if ~isfile(gtFile) || ~isfile(estFile)
    fprintf('Demo CSV files not found. Generating synthetic example...\n');
    n = 500; dt = 0.1; t = (0:n-1)' * dt;
    x  = 0.5*t + 1.5*sin(0.15*t);
    y  = 1.2*cos(0.11*t);
    yw = atan2(gradient(y, dt), gradient(x, dt));
    gt = table(t, x, y, yw, 'VariableNames', {'t','x','y','yaw'});
    writetable(gt, gtFile);

    est = gt;
    est.t   = est.t + 0.005*sin(0.04*est.t);
    est.x   = est.x + 0.03*est.t + 0.05*randn(size(est.x));
    est.y   = est.y - 0.02*est.t + 0.05*randn(size(est.y));
    est.yaw = est.yaw + deg2rad(1.5)*sin(0.2*est.t) + deg2rad(0.8)*randn(size(est.yaw));
    writetable(est, estFile);
end

gt  = readtable(gtFile);
est = readtable(estFile);

% Interpolate GT onto estimator timestamps
tq = est.t;
gt_i.x = interp1(gt.t, gt.x, tq, 'linear');
gt_i.y = interp1(gt.t, gt.y, tq, 'linear');
c = interp1(gt.t, cos(gt.yaw), tq, 'linear');
s = interp1(gt.t, sin(gt.yaw), tq, 'linear');
gt_i.yaw = atan2(s, c);

% 2D rigid alignment (Kabsch without scale)
E = [est.x, est.y];
G = [gt_i.x, gt_i.y];
muE = mean(E,1); muG = mean(G,1);
H = (E - muE)' * (G - muG);
[U,~,V] = svd(H);
R = V * U';
if det(R) < 0
    V(:,end) = -V(:,end);
    R = V * U';
end
tvec = muG' - R * muE';
E_aligned = (R * E')' + tvec';

% ATE
e_xy = E_aligned - G;
ate_rmse = sqrt(mean(sum(e_xy.^2,2)));

% RPE for delta_t = 1s
delta_t = 1.0;
e_rpe = [];
e_yaw = [];
for i = 1:length(tq)
    j = find(tq >= tq(i) + delta_t, 1, 'first');
    if isempty(j), continue; end

    d_gt = relSE2([gt_i.x(i), gt_i.y(i), gt_i.yaw(i)], [gt_i.x(j), gt_i.y(j), gt_i.yaw(j)]);
    d_es = relSE2([E_aligned(i,1), E_aligned(i,2), est.yaw(i)], [E_aligned(j,1), E_aligned(j,2), est.yaw(j)]);

    diff = d_es - d_gt;
    diff(3) = wrapAngle(diff(3));
    e_rpe(end+1,1) = norm(diff(1:2)); %#ok<AGROW>
    e_yaw(end+1,1) = abs(diff(3)); %#ok<AGROW>
end

rpe_rmse = sqrt(mean(e_rpe.^2));
rpe_yaw_mean = mean(e_yaw);

fprintf('ATE_trans_rmse_m = %.6f\n', ate_rmse);
fprintf('RPE_trans_rmse_m = %.6f\n', rpe_rmse);
fprintf('RPE_yaw_mean_rad = %.6f\n', rpe_yaw_mean);

% Normalized benchmark score
capATE = 2.0; capRPE = 0.8;
score = 100 * mean([max(0, 1 - ate_rmse/capATE), max(0, 1 - rpe_rmse/capRPE)]);
fprintf('Normalized score = %.2f / 100\n', score);

% Plots
figure;
plot(gt.x, gt.y, 'LineWidth', 1.5); hold on;
plot(E_aligned(:,1), E_aligned(:,2), 'LineWidth', 1.2);
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
legend('Ground truth','Estimate (aligned)');
title('Trajectory comparison');

figure;
plot(tq, sqrt(sum(e_xy.^2,2)), 'LineWidth', 1.2); grid on;
xlabel('time [s]'); ylabel('error [m]');
title('Position error over time');

% ---- local functions ----
function d = relSE2(p0, p1)
    % p = [x y yaw]
    dxw = p1(1) - p0(1);
    dyw = p1(2) - p0(2);
    c = cos(p0(3)); s = sin(p0(3));
    dx = c*dxw + s*dyw;
    dy = -s*dxw + c*dyw;
    dth = wrapAngle(p1(3) - p0(3));
    d = [dx, dy, dth];
end

function a = wrapAngle(a)
    a = mod(a + pi, 2*pi) - pi;
end
