% Chapter9_Lesson3.m
% Feature-Based Mapping (Landmarks) with per-landmark EKF (pose assumed known).
% This script simulates a 2D robot trajectory, range-bearing measurements, performs
% nearest-neighbor association with gating, initializes new landmarks, and runs EKF updates.
%
% Toolboxes (optional but relevant):
% - Robotics System Toolbox (for general robotics workflows)
% - Sensor Fusion and Tracking Toolbox (for EKF utilities, tracking objects)

clear; clc; close all;
rng(7);

wrap = @(a) mod(a+pi, 2*pi) - pi;

h_rb = @(x,m) [ ...
    hypot(m(1)-x(1), m(2)-x(2)); ...
    wrap(atan2(m(2)-x(2), m(1)-x(1)) - x(3)) ];

H_rb = @(x,m) jacobian_rb(x,m);

inv_h = @(x,z) [ x(1) + z(1)*cos(x(3)+z(2)); ...
                 x(2) + z(1)*sin(x(3)+z(2)) ];

% True landmarks
Mtrue = [ -6  5;
          -2 -4;
           4  6;
           7 -2;
           1  1.5;
          -7 -6 ];

% Trajectory (known poses for mapping)
T = 180;
t = linspace(0, 2*pi, T);
px = 2.5*cos(t) + 0.5*cos(3*t);
py = 2.0*sin(t);
dpx = gradient(px); dpy = gradient(py);
th = wrap(atan2(dpy, dpx));
X = [px(:), py(:), th(:)];

% Sensor
rmax = 9.0;
sigma_r = 0.15;
sigma_b = deg2rad(2.0);
R = diag([sigma_r^2, sigma_b^2]);
gate = 9.21; % chi2 gate for 2 dof ~99%

% Map storage
mu = zeros(0,2);
P  = zeros(2,2,0);

for k = 1:T
    x = X(k,:);
    for i = 1:size(Mtrue,1)
        z = h_rb(x, Mtrue(i,:));
        if z(1) <= rmax
            z = z + [sigma_r*randn; sigma_b*randn];
            z(2) = wrap(z(2));

            [j, bestd2] = associate(mu, P, x, z, R, gate, h_rb, H_rb, wrap);
            if isnan(j)
                mu(end+1,:) = inv_h(x,z)';
                P(:,:,end+1) = eye(2)*(1.5^2);
            else
                [mu(j,:), P(:,:,j)] = ekf_update_landmark(mu(j,:), P(:,:,j), x, z, R, h_rb, H_rb, wrap);
            end
        end
    end
end

% Plot
figure; hold on; grid on; axis equal;
plot(X(:,1), X(:,2), '-', 'DisplayName','robot path (known)');
scatter(Mtrue(:,1), Mtrue(:,2), 80, 'x', 'DisplayName','true landmarks');
scatter(mu(:,1), mu(:,2), 36, 'o', 'DisplayName','estimated landmarks');

% covariance ellipses (2-sigma) for first few landmarks
for j = 1:min(size(mu,1), 12)
    [V,D] = eig(P(:,:,j));
    w = max(diag(D), 1e-12);
    ang = linspace(0, 2*pi, 60);
    ell = V * (2*sqrt(w) .* [cos(ang); sin(ang)]);
    plot(mu(j,1)+ell(1,:), mu(j,2)+ell(2,:), 'LineWidth', 1);
end

xlabel('x [m]'); ylabel('y [m]');
title('Feature-Based Mapping with per-landmark EKF');
legend('Location','best');

% --------- local functions ---------
function H = jacobian_rb(x,m)
    dx = m(1)-x(1); dy = m(2)-x(2);
    q = dx^2 + dy^2;
    r = sqrt(q);
    if r < 1e-9
        r = 1e-9; q = r^2;
    end
    H = [ dx/r, dy/r;
         -dy/q, dx/q ];
end

function [j, bestd2] = associate(mu, P, x, z, R, gate, h_rb, H_rb, wrap)
    if isempty(mu)
        j = NaN; bestd2 = Inf; return;
    end
    bestd2 = Inf; j = NaN;
    for t = 1:size(mu,1)
        H = H_rb(x, mu(t,:));
        zhat = h_rb(x, mu(t,:));
        y = [z(1)-zhat(1); wrap(z(2)-zhat(2))];
        S = H*P(:,:,t)*H' + R;
        d2 = y'*(S\y);
        if d2 < bestd2
            bestd2 = d2; j = t;
        end
    end
    if bestd2 > gate
        j = NaN;
    end
end

function [mu_new, P_new] = ekf_update_landmark(mu, P, x, z, R, h_rb, H_rb, wrap)
    H = H_rb(x, mu);
    zhat = h_rb(x, mu);
    y = [z(1)-zhat(1); wrap(z(2)-zhat(2))];
    S = H*P*H' + R;
    K = P*H'/S;
    mu_new = (mu(:) + K*y).';
    P_new = (eye(2) - K*H)*P;
    P_new = 0.5*(P_new + P_new.');
end
