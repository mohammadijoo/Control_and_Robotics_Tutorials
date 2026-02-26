% Chapter20_Lesson3.m
% Localization + Mapping Integration (Capstone AMR)
% MATLAB / Simulink-oriented educational script:
%   - EKF localization with landmark updates
%   - Occupancy-grid log-odds mapping with ray updates
%
% Related toolboxes (optional for extension):
%   Navigation Toolbox, Robotics System Toolbox, Sensor Fusion and Tracking Toolbox

clear; clc; rng(6);

%% Configuration
dt = 0.1;
qv = 0.02; qw = 0.03;
rr = 0.15; rb = 0.03;
gridRes = 0.2; gridSize = 120;
lOcc = 0.85; lFree = -0.40; lMin = -4; lMax = 4;

%% State and covariance
xhat = [1.0; 1.0; 0.0];
P = diag([0.05, 0.05, 0.02]);

%% Ground truth and map
xtrue = [1.0; 1.0; 0.0];
landmarks = [2.0,10.0; 10.0,2.5; 9.5,10.5; 2.0,5.5];
L = zeros(gridSize, gridSize); % log-odds grid

trajTrue = zeros(240,3);
trajEst  = zeros(240,3);

%% Main integration loop
for k = 1:240
    vCmd = 0.35 + 0.05*sin(0.05*k);
    wCmd = 0.25*sin(0.03*k);

    % ----- Truth propagation -----
    xtrue(1) = xtrue(1) + vCmd*cos(xtrue(3))*dt;
    xtrue(2) = xtrue(2) + vCmd*sin(xtrue(3))*dt;
    xtrue(3) = wrapToPiLocal(xtrue(3) + wCmd*dt + 0.005*randn);

    % ----- EKF prediction -----
    vMeas = vCmd + 0.02*randn;
    wMeas = wCmd + 0.01*randn;
    th = xhat(3);

    xhat(1) = xhat(1) + vMeas*cos(th)*dt;
    xhat(2) = xhat(2) + vMeas*sin(th)*dt;
    xhat(3) = wrapToPiLocal(xhat(3) + wMeas*dt);

    F = [1,0,-vMeas*sin(th)*dt;
         0,1, vMeas*cos(th)*dt;
         0,0,1];
    G = [cos(th)*dt, 0;
         sin(th)*dt, 0;
         0, dt];
    Q = diag([qv^2, qw^2]);
    P = F*P*F' + G*Q*G';

    % ----- Landmark corrections every 5 steps -----
    if mod(k,5) == 0
        for i = 1:size(landmarks,1)
            dx = landmarks(i,1) - xtrue(1);
            dy = landmarks(i,2) - xtrue(2);
            zRange = hypot(dx,dy) + 0.08*randn;
            if zRange > 7.5
                continue;
            end
            zBear = wrapToPiLocal(atan2(dy,dx) - xtrue(3) + 0.02*randn);

            dxh = landmarks(i,1) - xhat(1);
            dyh = landmarks(i,2) - xhat(2);
            q = dxh^2 + dyh^2;
            if q < 1e-12
                continue;
            end

            zhat = [sqrt(q); wrapToPiLocal(atan2(dyh,dxh) - xhat(3))];
            H = [-dxh/sqrt(q), -dyh/sqrt(q), 0;
                  dyh/q,       -dxh/q,      -1];
            R = diag([rr^2, rb^2]);

            S = H*P*H' + R;
            innov = [zRange - zhat(1); wrapToPiLocal(zBear - zhat(2))];
            d2 = innov' * (S \ innov);

            if d2 < 9.21
                K = P*H'/S;
                xhat = xhat + K*innov;
                xhat(3) = wrapToPiLocal(xhat(3));
                I = eye(3);
                P = (I-K*H)*P*(I-K*H)' + K*R*K'; % Joseph form
            end
        end
    end

    % ----- Mapping update with three synthetic rays (educational simplification) -----
    beamAngles = [-0.7, 0.0, 0.7];
    for a = beamAngles
        rrHit = 5.0;
        ex = xhat(1) + rrHit*cos(xhat(3)+a);
        ey = xhat(2) + rrHit*sin(xhat(3)+a);
        [L] = updateRayLogOdds(L, xhat(1), xhat(2), ex, ey, true, ...
                               gridRes, lOcc, lFree, lMin, lMax);
    end

    trajTrue(k,:) = xtrue.';
    trajEst(k,:)  = xhat.';
end

%% Metrics
posRMSE = sqrt(mean(sum((trajTrue(:,1:2)-trajEst(:,1:2)).^2, 2)));
yawErr = wrapToPiVector(trajTrue(:,3)-trajEst(:,3));
yawRMSE = sqrt(mean(yawErr.^2));

occProb = 1 ./ (1 + exp(-L));
fprintf('Final estimated pose: [%.4f %.4f %.4f]\n', xhat(1), xhat(2), xhat(3));
fprintf('Final covariance diag: [%.5f %.5f %.5f]\n', P(1,1), P(2,2), P(3,3));
fprintf('Position RMSE [m]: %.4f\n', posRMSE);
fprintf('Yaw RMSE [rad]: %.4f\n', yawRMSE);
fprintf('Map occupancy probability stats: min=%.3f max=%.3f mean=%.3f\n', ...
        min(occProb(:)), max(occProb(:)), mean(occProb(:)));

%% Optional quick visualization
figure;
imagesc(occProb); axis image; colorbar;
title('Occupancy Probability Map (estimated-pose updates)');
xlabel('grid x'); ylabel('grid y');

figure;
plot(trajTrue(:,1), trajTrue(:,2), 'LineWidth', 1.2); hold on;
plot(trajEst(:,1),  trajEst(:,2),  '--', 'LineWidth', 1.2);
axis equal; grid on; legend('True', 'Estimated');
title('Trajectory: truth vs EKF estimate');
xlabel('x [m]'); ylabel('y [m]');

%% Local functions
function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end

function v = wrapToPiVector(v)
    v = mod(v + pi, 2*pi) - pi;
end

function [L] = updateRayLogOdds(L, rx, ry, ex, ey, hit, gridRes, lOcc, lFree, lMin, lMax)
    [g0x, g0y] = worldToGrid(rx, ry, gridRes);
    [g1x, g1y] = worldToGrid(ex, ey, gridRes);
    [H, W] = size(L);
    if ~inBounds(g0x, g0y, W, H) || ~inBounds(g1x, g1y, W, H)
        return;
    end

    ray = bresenham(g0x, g0y, g1x, g1y);
    if size(ray,1) >= 2
        for i = 1:size(ray,1)-1
            cx = ray(i,1); cy = ray(i,2);
            L(cy, cx) = clip(L(cy, cx) + lFree, lMin, lMax);
        end
    end
    if hit
        cx = ray(end,1); cy = ray(end,2);
        L(cy, cx) = clip(L(cy, cx) + lOcc, lMin, lMax);
    end
end

function [gx, gy] = worldToGrid(x, y, gridRes)
    gx = floor(x/gridRes) + 1;
    gy = floor(y/gridRes) + 1;
end

function ok = inBounds(gx, gy, W, H)
    ok = gx >= 1 && gx <= W && gy >= 1 && gy <= H;
end

function pts = bresenham(x0, y0, x1, y1)
    dx = abs(x1 - x0); sx = sign(x1 - x0); if sx == 0, sx = 1; end
    dy = -abs(y1 - y0); sy = sign(y1 - y0); if sy == 0, sy = 1; end
    err = dx + dy;
    x = x0; y = y0;
    pts = [x, y];
    while ~(x == x1 && y == y1)
        e2 = 2*err;
        if e2 >= dy
            err = err + dy;
            x = x + sx;
        end
        if e2 <= dx
            err = err + dx;
            y = y + sy;
        end
        pts = [pts; x, y]; %#ok<AGROW>
    end
end

function y = clip(x, lo, hi)
    y = min(max(x, lo), hi);
end
