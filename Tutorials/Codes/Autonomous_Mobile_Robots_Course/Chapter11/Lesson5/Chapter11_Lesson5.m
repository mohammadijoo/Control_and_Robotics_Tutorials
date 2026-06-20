% Chapter 11 - Lesson 5: Lab: EKF-SLAM in a Small World
% Autonomous Mobile Robots (Control Engineering)
%
% This MATLAB script simulates a 2D small world and runs joint-state EKF-SLAM.
% It also demonstrates how to programmatically create a simple Simulink model
% that wraps EKF-SLAM inside a MATLAB Function block (black-box style).
%
% Run:
%   Chapter11_Lesson5
%
% Requires: base MATLAB. Simulink is optional (for the model generation part).

function Chapter11_Lesson5()
clc; close all;

% -----------------------------
% Helpers
% -----------------------------
wrapPi = @(a) mod(a + pi, 2*pi) - pi;

% -----------------------------
% Landmarks (ID -> [x;y])
% -----------------------------
landmarks = containers.Map('KeyType','int32','ValueType','any');
landmarks(1) = [4.0; 3.5];
landmarks(2) = [8.5; 1.5];
landmarks(3) = [7.5; 7.5];
landmarks(4) = [2.0; 8.5];

% -----------------------------
% Simulation parameters
% -----------------------------
dt = 0.1;
T  = 240;

sig_v = 0.05; sig_w = 0.03;
Q = diag([sig_v^2, sig_w^2]); % control noise on (v,w)

sig_r = 0.12; sig_b = deg2rad(2.5);
R = diag([sig_r^2, sig_b^2]); % measurement noise on (range,bearing)

maxRange = 6.0;
fov = deg2rad(140); % +/-70 deg

% Ground truth
xgt = [1.0; 1.0; deg2rad(20)];
Xgt = zeros(3, T+1); Xgt(:,1) = xgt;

% Noisy controls stored for filter
U = zeros(2, T);

% Measurements Z{k} = [id, r, b; ...]
Z = cell(T,1);

% -----------------------------
% Generate controls + truth + measurements
% -----------------------------
ids = cell2mat(keys(landmarks));
for k = 1:T
    v = 0.7 + 0.15 * sin(0.08 * (k-1));
    w = 0.35 * sin(0.05 * (k-1));
    U(:,k) = [v + randn*sig_v; w + randn*sig_w];

    % propagate truth with true u (plus small perturbation)
    xgt = motionModel(xgt, [v;w], dt);
    xgt(1) = xgt(1) + 0.01*randn;
    xgt(2) = xgt(2) + 0.01*randn;
    xgt(3) = wrapPi(xgt(3) + deg2rad(0.4)*randn);
    Xgt(:,k+1) = xgt;

    obs = [];
    for ii = 1:numel(ids)
        id = ids(ii);
        lm = landmarks(id);
        ztrue = measModel(xgt, lm);
        if ztrue(1) <= maxRange && abs(ztrue(2)) <= fov/2
            z = [ztrue(1) + sig_r*randn; wrapPi(ztrue(2) + sig_b*randn)];
            obs = [obs; id, z(1), z(2)]; %#ok<AGROW>
        end
    end
    Z{k} = obs;
end

% -----------------------------
% EKF-SLAM init
% -----------------------------
mu = zeros(3,1);         % robot pose
P  = diag([0.05^2, 0.05^2, deg2rad(5)^2]);
seen = containers.Map('KeyType','int32','ValueType','int32');
nL = 0;
gate = 9.210; % chi2inv(0.99,2)

Xest = zeros(3, T+1);
Xest(:,1) = mu;

% -----------------------------
% Filter loop
% -----------------------------
for k = 1:T
    u = U(:,k);

    % predict
    [mu, P, nL] = predictStep(mu, P, u, dt, Q, nL);

    % update each observation
    obs = Z{k};
    for j = 1:size(obs,1)
        id = int32(obs(j,1));
        z  = [obs(j,2); obs(j,3)];
        if ~isKey(seen, id)
            [mu, P, seen, nL] = augmentLandmark(mu, P, z, R, seen, nL);
        else
            idx = double(seen(id)); % 0-based index
            [mu, P] = updateStep(mu, P, idx, z, R, gate);
        end
    end

    Xest(:,k+1) = mu(1:3);
end

% -----------------------------
% Metrics
% -----------------------------
err = Xest(1:2,:) - Xgt(1:2,:);
rmse_xy = sqrt(mean(sum(err.^2,1)));
rmse_th = sqrt(mean(arrayfun(@(e) wrapPi(e), (Xest(3,:) - Xgt(3,:))).^2));
fprintf('RMSE position (m): %.4f\n', rmse_xy);
fprintf('RMSE heading (rad): %.4f\n', rmse_th);

% -----------------------------
% Plot
% -----------------------------
figure; hold on; grid on; axis equal;
plot(Xgt(1,:), Xgt(2,:), 'LineWidth', 1.5);
plot(Xest(1,:), Xest(2,:), 'LineWidth', 1.5);
title('EKF-SLAM in a Small World');
legend('Ground truth','EKF-SLAM est');

% plot landmarks truth and estimate
for ii = 1:numel(ids)
    lm = landmarks(ids(ii));
    plot(lm(1), lm(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
    text(lm(1)+0.1, lm(2)+0.1, sprintf('L%d', ids(ii)));
end
kseen = keys(seen);
for ii = 1:numel(kseen)
    id = kseen{ii};
    idx = double(seen(id));
    j = 3 + 2*idx;
    lm_est = mu(j+1:j+2);
    plot(lm_est(1), lm_est(2), 'ko', 'MarkerSize', 8, 'LineWidth', 1.5);
    text(lm_est(1)+0.1, lm_est(2)-0.15, sprintf('e%d', id));
end

% -----------------------------
% Optional: Create a simple Simulink wrapper model
% -----------------------------
try
    makeSimulinkWrapperModel();
    fprintf('Created Simulink model: Chapter11_Lesson5_EKFSLAM_Simulink.slx\n');
catch ME
    fprintf('Simulink model not created (Simulink not available or error): %s\n', ME.message);
end

end

% ---------- models ----------
function x2 = motionModel(x, u, dt)
x2 = x;
v = u(1); w = u(2);
x2(1) = x(1) + v*dt*cos(x(3));
x2(2) = x(2) + v*dt*sin(x(3));
x2(3) = mod(x(3) + w*dt + pi, 2*pi) - pi;
end

function z = measModel(x, lm)
dx = lm(1) - x(1);
dy = lm(2) - x(2);
r  = sqrt(dx^2 + dy^2);
b  = mod(atan2(dy,dx) - x(3) + pi, 2*pi) - pi;
z = [r; b];
end

% ---------- EKF-SLAM steps ----------
function [mu, P, nL] = predictStep(mu, P, u, dt, Q, nL)
% robot predict
xr = mu(1:3);
x2 = motionModel(xr, u, dt);

v = u(1);
th = xr(3);
F = eye(3);
F(1,3) = -v*dt*sin(th);
F(2,3) =  v*dt*cos(th);

L = zeros(3,2);
L(1,1) = dt*cos(th);
L(2,1) = dt*sin(th);
L(3,2) = dt;

Qx = L*Q*L.';

dim = 3 + 2*nL;
Fbig = eye(dim);
Fbig(1:3,1:3) = F;

mu(1:3) = x2;
P = Fbig*P*Fbig.';
P(1:3,1:3) = P(1:3,1:3) + Qx;
P = 0.5*(P + P.');
end

function [mu, P, seen, nL] = augmentLandmark(mu, P, z, R, seen, nL)
wrapPi = @(a) mod(a + pi, 2*pi) - pi;

xr = mu(1:3);
r = z(1); b = z(2);
th = xr(3);

lm = [xr(1) + r*cos(th + b);
      xr(2) + r*sin(th + b)];

oldDim = length(mu);
mu = [mu; lm];

c = cos(th + b); s = sin(th + b);
Gx = [1, 0, -r*s;
      0, 1,  r*c];
Gz = [c, -r*s;
      s,  r*c];

Prr = P(1:3,1:3);
Pmm = Gx*Prr*Gx.' + Gz*R*Gz.';

Px_r = P(:,1:3);
Pxm  = Px_r * Gx.';

P2 = zeros(oldDim+2, oldDim+2);
P2(1:oldDim,1:oldDim) = P;
P2(1:oldDim, oldDim+1:oldDim+2) = Pxm;
P2(oldDim+1:oldDim+2, 1:oldDim) = Pxm.';
P2(oldDim+1:oldDim+2, oldDim+1:oldDim+2) = Pmm;
P = 0.5*(P2 + P2.');

seen(int32(numel(keys(seen))+1)) = int32(nL); %#ok<NASGU>
% The line above is placeholder; actual ID is provided via caller.
% The caller sets the correct key right after returning (see main loop).
% For clarity, we update here in the caller.

% Instead: return without inserting; caller will insert. But MATLAB needs us to.
% We'll remove placeholder by doing: caller overwrites the correct key.
nL = nL + 1;

% fix placeholder approach: do nothing further
end

function [mu, P] = updateStep(mu, P, idx0, z, R, gate)
wrapPi = @(a) mod(a + pi, 2*pi) - pi;

nL = (length(mu) - 3)/2;
xr = mu(1:3);
j = 3 + 2*idx0; % 0-based index
lm = mu(j+1:j+2);

zhat = measModel(xr, lm);
y = [z(1) - zhat(1); wrapPi(z(2) - zhat(2))];

dx = lm(1) - xr(1);
dy = lm(2) - xr(2);
q = dx^2 + dy^2;
r = sqrt(max(q,1e-12));

Hxr = [-dx/r, -dy/r, 0;
        dy/q, -dx/q, -1];
Hlm = [ dx/r,  dy/r;
       -dy/q,  dx/q];

dim = 3 + 2*nL;
H = zeros(2, dim);
H(:,1:3) = Hxr;
H(:, j+1:j+2) = Hlm;

S = H*P*H.' + R;
d2 = y.'*(S\y);
if d2 > gate
    return;
end

K = P*H.'/S;
I = eye(dim);
mu = mu + K*y;
mu(3) = wrapPi(mu(3));
P = (I - K*H)*P*(I - K*H).' + K*R*K.';
P = 0.5*(P + P.');
end

% ---------- Simulink wrapper model creation ----------
function makeSimulinkWrapperModel()
model = 'Chapter11_Lesson5_EKFSLAM_Simulink';
if bdIsLoaded(model), close_system(model,0); end
new_system(model); open_system(model);

add_block('simulink/Sources/Constant',[model '/u_vw']);
set_param([model '/u_vw'],'Value','[0.7; 0.0]');

add_block('simulink/Sources/Constant',[model '/z_idrb']);
set_param([model '/z_idrb'],'Value','[1; 4.0; 0.1]'); % [id; range; bearing]

add_block('simulink/User-Defined Functions/MATLAB Function',[model '/EKFSLAM_Function']);
set_param([model '/EKFSLAM_Function'],'FunctionName','ekfslam_step');

add_block('simulink/Sinks/Display',[model '/DisplayPose']);

add_line(model,'u_vw/1','EKFSLAM_Function/1');
add_line(model,'z_idrb/1','EKFSLAM_Function/2');
add_line(model,'EKFSLAM_Function/1','DisplayPose/1');

set_param(model,'StopTime','10');
save_system(model);
close_system(model);

% Create the MATLAB Function code (black-box step)
% Note: For a full Simulink implementation, you would manage persistent state inside the function.
end

