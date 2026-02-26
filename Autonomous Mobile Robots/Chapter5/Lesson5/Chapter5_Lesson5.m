% Chapter5_Lesson5.m
%
% Lab: Characterizing Odometry Error (Differential Drive)
%
% This MATLAB script:
% 1) Loads a CSV log: t,nL,nR,x_odom,y_odom,th_odom,x_gt,y_gt,th_gt,(optional)trial_id
% 2) Computes endpoint error, trajectory RMSE, drift-per-meter (logged odom)
% 3) Calibrates (rL, rR, b) via Gauss-Newton least squares on increment residuals
% 4) Reconstructs odometry from encoders with calibrated parameters and re-evaluates metrics
% 5) Estimates increment residual covariance Q_hat
%
% Usage:
%   results = Chapter5_Lesson5("your_log.csv", 4096, 0.05, 0.30, 1.0);
%
% Recommended toolboxes (optional):
% - Optimization Toolbox
% - Robotics System Toolbox (for additional plotting/poses)
%
function results = Chapter5_Lesson5(csvPath, ticksPerRev, r_nom, b_nom, w_theta)

if nargin < 5
    w_theta = 1.0;
end
if nargin < 4
    b_nom = 0.30;
end
if nargin < 3
    r_nom = 0.05;
end

T = readtable(csvPath);

req = {'t','nL','nR','x_odom','y_odom','th_odom','x_gt','y_gt','th_gt'};
for i=1:numel(req)
    if ~ismember(req{i}, T.Properties.VariableNames)
        error("Missing required column: %s", req{i});
    end
end

if ~ismember('trial_id', T.Properties.VariableNames)
    T.trial_id = zeros(height(T),1);
end

T.th_odom = wrapToPi(T.th_odom);
T.th_gt   = wrapToPi(T.th_gt);

trialIds = unique(T.trial_id);

% Metrics: logged odom
M_logged = [];
for i=1:numel(trialIds)
    tid = trialIds(i);
    G = T(T.trial_id == tid,:);
    m = compute_metrics_table(G);
    m.trial_id = tid;
    M_logged = [M_logged; struct2table(m)];
end

% Calibrate on all data concatenated
p0 = [r_nom; r_nom; b_nom];
p_hat = gauss_newton_calibrate(T, ticksPerRev, p0, w_theta);

% Metrics: reconstructed with calibrated params
M_cal = [];
for i=1:numel(trialIds)
    tid = trialIds(i);
    G = T(T.trial_id == tid,:);
    G2 = reconstruct_odom_from_enc(G, ticksPerRev, p_hat);
    m2 = compute_metrics_table(G2);
    m2.trial_id = tid;
    M_cal = [M_cal; struct2table(m2)];
end

% Increment residuals and Q_hat after calibration
eps_all = [];
for i=1:numel(trialIds)
    tid = trialIds(i);
    G = T(T.trial_id == tid,:);
    eps = increment_residuals(G, ticksPerRev, p_hat);
    if ~isempty(eps)
        eps_all = [eps_all; eps];
    end
end

if isempty(eps_all)
    eps_mean = [0 0 0];
    Q_hat = nan(3,3);
else
    eps_mean = mean(eps_all, 1);
    E0 = eps_all - eps_mean;
    Q_hat = cov(E0, 1) * (size(E0,1)/(size(E0,1)-1)); % unbiased
end

results = struct();
results.p_hat = p_hat;
results.metrics_logged = M_logged;
results.metrics_calibrated = M_cal;
results.increment_residual_mean = eps_mean;
results.Q_hat = Q_hat;

disp("=== Calibration Result ===");
fprintf("rL_hat = %.10f m\n", p_hat(1));
fprintf("rR_hat = %.10f m\n", p_hat(2));
fprintf("b_hat  = %.10f m\n", p_hat(3));
disp(" ");

disp("=== Metrics (logged odom) ===");
disp(M_logged);

disp("=== Metrics (reconstructed, calibrated) ===");
disp(M_cal);

disp("=== Increment residual mean [dx dy dth] ===");
disp(eps_mean);

disp("=== Q_hat ===");
disp(Q_hat);

% Optional plots
try
    figure;
    plot(M_logged.trial_id, M_logged.end_pos, '-o'); hold on;
    plot(M_cal.trial_id, M_cal.end_pos, '-o');
    xlabel('trial\_id'); ylabel('endpoint position error (m)');
    legend('logged','calibrated'); title('Endpoint position error per trial');
    grid on;

    figure;
    plot(M_logged.trial_id, M_logged.rmse_pos, '-o'); hold on;
    plot(M_cal.trial_id, M_cal.rmse_pos, '-o');
    xlabel('trial\_id'); ylabel('trajectory RMSE pos (m)');
    legend('logged','calibrated'); title('Trajectory RMSE position per trial');
    grid on;
catch
    % ignore plotting errors in headless mode
end

end

% -----------------------------
% Helpers
% -----------------------------

function m = compute_metrics_table(G)
ex = G.x_odom - G.x_gt;
ey = G.y_odom - G.y_gt;
eth = wrapToPi(G.th_odom - G.th_gt);

epos = sqrt(ex.^2 + ey.^2);
m.rmse_pos = sqrt(mean(epos.^2));
m.rmse_th  = sqrt(mean(eth.^2));

m.end_pos = sqrt(ex(end)^2 + ey(end)^2);
m.end_th  = abs(eth(end));

dxg = diff(G.x_gt);
dyg = diff(G.y_gt);
s_total = sum(sqrt(dxg.^2 + dyg.^2));
m.s_total = s_total;

if s_total > 1e-12
    m.drift_per_m = m.end_pos / s_total;
else
    m.drift_per_m = NaN;
end
end

function dphi = ticks_to_dphi(n, ticksPerRev)
dn = [0; diff(n)];
dphi = (2*pi / ticksPerRev) * dn;
end

function inc = predict_increment(th_prev, dphiL, dphiR, p)
rL = p(1); rR = p(2); b = p(3);
sL = rL * dphiL;
sR = rR * dphiR;
ds = 0.5*(sR + sL);
dth = (sR - sL)/b;
th_mid = wrapToPi(th_prev + 0.5*dth);
dx = ds .* cos(th_mid);
dy = ds .* sin(th_mid);
inc = [dx dy dth];
end

function G2 = reconstruct_odom_from_enc(G, ticksPerRev, p_hat)
nL = G.nL; nR = G.nR;
dphiL = ticks_to_dphi(nL, ticksPerRev);
dphiR = ticks_to_dphi(nR, ticksPerRev);

th_gt = G.th_gt;
th_prev = [th_gt(1); th_gt(1:end-1)];

inc = predict_increment(th_prev, dphiL, dphiR, p_hat);

x0 = [G.x_gt(1), G.y_gt(1), G.th_gt(1)];
X = zeros(height(G),3);
X(1,:) = x0;
X(2:end,:) = cumsum(inc(2:end,:),1) + x0;
X(:,3) = wrapToPi(X(:,3));

G2 = G;
G2.x_odom = X(:,1);
G2.y_odom = X(:,2);
G2.th_odom = X(:,3);
end

function eps = increment_residuals(G, ticksPerRev, p_hat)
nL = G.nL; nR = G.nR;
dphiL = ticks_to_dphi(nL, ticksPerRev);
dphiR = ticks_to_dphi(nR, ticksPerRev);

th_gt = G.th_gt;
th_prev = [th_gt(1); th_gt(1:end-1)];

inc_pred = predict_increment(th_prev, dphiL, dphiR, p_hat);

dx = [0; diff(G.x_gt)];
dy = [0; diff(G.y_gt)];
dth = wrapToPi([0; diff(G.th_gt)]);
inc_gt = [dx dy dth];

eps = inc_pred(2:end,:) - inc_gt(2:end,:);
end

function p_hat = gauss_newton_calibrate(T, ticksPerRev, p0, w_theta)
p = p0(:);
maxIter = 15;
lambda = 1e-9;

for it=1:maxIter
    [r, J] = residual_and_jacobian(T, ticksPerRev, p, w_theta);
    A = J'*J + lambda*eye(3);
    g = J'*r;
    dp = -A\g;

    if norm(dp) < 1e-12
        break;
    end

    alpha = 1.0;
    for ls=1:10
        cand = p + alpha*dp;
        if all(cand > 0)
            p = cand;
            break;
        end
        alpha = 0.5*alpha;
    end
end
p_hat = p;
end

function [r, J] = residual_and_jacobian(T, ticksPerRev, p, w_theta)
nL = T.nL; nR = T.nR;
dphiL = ticks_to_dphi(nL, ticksPerRev);
dphiR = ticks_to_dphi(nR, ticksPerRev);

th_gt = T.th_gt;
th_prev = [th_gt(1); th_gt(1:end-1)];

dx_gt = [0; diff(T.x_gt)];
dy_gt = [0; diff(T.y_gt)];
dth_gt = wrapToPi([0; diff(T.th_gt)]);

rL = p(1); rR = p(2); b = p(3);

sL = rL * dphiL;
sR = rR * dphiR;
ds = 0.5*(sR + sL);
dth = (sR - sL)/b;

th_mid = wrapToPi(th_prev + 0.5*dth);

dx = ds .* cos(th_mid);
dy = ds .* sin(th_mid);

rx = dx - dx_gt;
ry = dy - dy_gt;
rth = wrapToPi(dth - dth_gt);

% skip first sample
rx = rx(2:end); ry = ry(2:end); rth = rth(2:end);
N = numel(rx);

r = zeros(3*N,1);
r(1:3:end) = rx;
r(2:3:end) = ry;
r(3:3:end) = w_theta * rth;

% Jacobian
dphiLk = dphiL(2:end);
dphiRk = dphiR(2:end);
sLk = sL(2:end);
sRk = sR(2:end);
dsk = ds(2:end);
ak = th_mid(2:end);

ca = cos(ak); sa = sin(ak);

dds_drL = 0.5*dphiLk;
dds_drR = 0.5*dphiRk;

ddth_drL = -(dphiLk)/b;
ddth_drR = (dphiRk)/b;
ddth_db  = -(sRk - sLk)/(b^2);

ddx_drL = ca.*dds_drL + dsk.*(-sa).*(0.5*ddth_drL);
ddx_drR = ca.*dds_drR + dsk.*(-sa).*(0.5*ddth_drR);
ddx_db  = dsk.*(-sa).*(0.5*ddth_db);

ddy_drL = sa.*dds_drL + dsk.*ca.*(0.5*ddth_drL);
ddy_drR = sa.*dds_drR + dsk.*ca.*(0.5*ddth_drR);
ddy_db  = dsk.*ca.*(0.5*ddth_db);

ddth_drL_s = w_theta*ddth_drL;
ddth_drR_s = w_theta*ddth_drR;
ddth_db_s  = w_theta*ddth_db;

J = zeros(3*N,3);
J(1:3:end,1) = ddx_drL;
J(1:3:end,2) = ddx_drR;
J(1:3:end,3) = ddx_db;

J(2:3:end,1) = ddy_drL;
J(2:3:end,2) = ddy_drR;
J(2:3:end,3) = ddy_db;

J(3:3:end,1) = ddth_drL_s;
J(3:3:end,2) = ddth_drR_s;
J(3:3:end,3) = ddth_db_s;
end
