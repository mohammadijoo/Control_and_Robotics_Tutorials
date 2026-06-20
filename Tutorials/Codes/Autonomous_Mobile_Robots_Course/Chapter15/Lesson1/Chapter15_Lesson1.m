% Chapter15_Lesson1.m
% Pure Pursuit and Geometric Path Tracking (MATLAB)
%
% This file contains:
%   1) A demo script (top-level) that simulates a kinematic bicycle model
%   2) Local functions for pure pursuit geometry
%
% Run:
%   Chapter15_Lesson1

clear; clc;

% Path: gentle sinusoid
xs = linspace(0, 25, 500)';
ys = 1.8 * sin(0.22 * xs);
path = [xs, ys];

pose = [-2.0; -2.0; 0.2]; % [x;y;theta]
dt = 0.02;
wheelbase = 0.33;
v = 1.5;

Ld_min = 0.8; Ld_max = 3.5; k_v = 0.8;
kappa_max = 1.6;
delta_max = deg2rad(32);

traj = zeros(round(25/dt), 2);
lookPts = zeros(round(25/dt), 2);

for k = 1:size(traj,1)
    Ld = min(Ld_max, max(Ld_min, Ld_min + k_v * abs(v)));
    [kappa, pClosest, pLook] = purePursuitCurvature(pose, path, Ld);
    kappa = min(kappa_max, max(-kappa_max, kappa));
    delta = atan(wheelbase * kappa);
    delta = min(delta_max, max(-delta_max, delta));

    pose = bicycleStep(pose, v, delta, wheelbase, dt);
    traj(k,:) = pose(1:2)';
    lookPts(k,:) = pLook';
end

figure;
plot(path(:,1), path(:,2)); hold on;
plot(traj(:,1), traj(:,2));
plot(lookPts(1:60:end,1), lookPts(1:60:end,2), '.');
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Pure Pursuit (kinematic bicycle)');
legend('path','robot','lookahead (every 60)');

% -------------------------- Local functions --------------------------

function a = wrapAngle(a)
    a = mod(a + pi, 2*pi) - pi;
end

function R = rot2(th)
    R = [cos(th) -sin(th); sin(th) cos(th)];
end

function pBody = worldToBody(pose, pWorld)
    th = pose(3);
    R = rot2(th)';
    t = pose(1:2);
    pBody = R * (pWorld - t);
end

function [s, seg] = polylineArcLength(path)
    d = diff(path,1,1);
    seg = sqrt(sum(d.^2,2));
    s = [0; cumsum(seg)];
end

function [q, t] = closestPointOnSegment(p, a, b)
    ab = b - a;
    denom = ab' * ab;
    if denom <= 1e-12
        q = a; t = 0;
        return;
    end
    t0 = ((p - a)' * ab) / denom;
    t = min(1, max(0, t0));
    q = a + t * ab;
end

function [qBest, iBest, tBest, dBest] = closestPointOnPolyline(p, path)
    dBest = inf;
    qBest = path(1,:)'; iBest = 1; tBest = 0;
    for i = 1:size(path,1)-1
        [q, t] = closestPointOnSegment(p, path(i,:)', path(i+1,:)');
        d = norm(p - q);
        if d < dBest
            dBest = d; qBest = q; iBest = i; tBest = t;
        end
    end
end

function p = pointAtArcLength(path, s, seg, sQuery)
    if sQuery <= 0
        p = path(1,:)'; return;
    end
    if sQuery >= s(end)
        p = path(end,:)'; return;
    end
    j = find(s <= sQuery, 1, 'last');
    ds = sQuery - s(j);
    t = ds / max(seg(j), 1e-12);
    p = (1-t) * path(j,:)' + t * path(j+1,:)';
end

function [kappa, pClosest, pLook] = purePursuitCurvature(pose, path, Ld)
    Ld = max(Ld, 1e-3);
    p = pose(1:2);
    [s, seg] = polylineArcLength(path);
    [q, i, t, ~] = closestPointOnPolyline(p, path);
    sClosest = s(i) + t * seg(i);
    pLook = pointAtArcLength(path, s, seg, sClosest + Ld);

    lookB = worldToBody(pose, pLook);
    L = norm(lookB);
    if L <= 1e-9
        kappa = 0.0;
    else
        kappa = 2.0 * lookB(2) / (L * L);
    end
    pClosest = q;
end

function poseN = bicycleStep(pose, v, delta, wheelbase, dt)
    x = pose(1) + v * cos(pose(3)) * dt;
    y = pose(2) + v * sin(pose(3)) * dt;
    th = wrapAngle(pose(3) + (v / wheelbase) * tan(delta) * dt);
    poseN = [x; y; th];
end
