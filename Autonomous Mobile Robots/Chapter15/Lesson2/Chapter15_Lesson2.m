% Chapter15_Lesson2.m
% Stanley Controller for Ground Vehicles (kinematic bicycle model, educational)
%
% Run:
%   Chapter15_Lesson2
%
% This script:
%   1) builds a smooth reference path (polyline)
%   2) tracks it using Stanley steering control
%   3) plots trajectory and tracking errors

function Chapter15_Lesson2
    % Reference path
    n = 400;
    xr = linspace(0, 50, n);
    yr = 2.5*sin(0.18*xr) + 1.0*sin(0.04*xr);

    % Parameters
    p.L = 2.7;
    p.k = 1.4;
    p.v0 = 0.5;
    p.maxSteer = deg2rad(32);

    dt = 0.02;  T = 25.0;
    steps = floor(T/dt);

    % State: [x;y;psi], constant speed v
    x = -2.0; y = 3.5; psi = deg2rad(-10);
    v = 6.0;

    xs = zeros(steps,1);
    ys = zeros(steps,1);
    eys = zeros(steps,1);
    epsis = zeros(steps,1);
    deltas = zeros(steps,1);

    for k = 1:steps
        [delta, e_y, e_psi] = stanley_control(x, y, psi, v, xr, yr, p);
        % Bicycle step
        x = x + v*cos(psi)*dt;
        y = y + v*sin(psi)*dt;
        psi = wrap_angle(psi + v/p.L * tan(delta) * dt);

        xs(k) = x; ys(k) = y;
        eys(k) = e_y; epsis(k) = e_psi;
        deltas(k) = delta;
    end

    % Plots
    figure; plot(xr, yr, 'LineWidth', 1.5); hold on;
    plot(xs, ys, 'LineWidth', 1.5);
    axis equal; grid on;
    legend('reference path','vehicle trajectory');
    title('Stanley Controller: path tracking');

    t = linspace(0, T, steps);
    figure; plot(t, eys, 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(epsis), 'LineWidth', 1.5);
    grid on; legend('e_y [m]','e_\psi [deg]');
    title('Tracking errors');

    figure; plot(t, rad2deg(deltas), 'LineWidth', 1.5);
    grid on; legend('\delta [deg]');
    title('Steering command');
end

function [delta, e_y, e_psi] = stanley_control(x, y, psi, v, xr, yr, p)
    % Find nearest path index (simple vertex scan)
    d2 = (xr - x).^2 + (yr - y).^2;
    [~, i] = min(d2);

    % Estimate reference heading using neighbors
    i0 = max(i-1, 1);
    i2 = min(i+1, numel(xr));
    psi_ref = atan2(yr(i2)-yr(i0), xr(i2)-xr(i0));

    % Convention: e_psi = psi_ref - psi
    e_psi = wrap_angle(psi_ref - psi);

    % Signed cross-track via 2D cross product
    tx = cos(psi_ref); ty = sin(psi_ref);
    vx = x - xr(i);   vy = y - yr(i);
    cross = tx*vy - ty*vx;
    e_y = sign_safe(cross) * hypot(vx, vy);

    delta = e_psi + atan2(p.k * e_y, v + p.v0);
    delta = min(max(delta, -p.maxSteer), p.maxSteer);
end

function s = sign_safe(a)
    if a >= 0, s = 1; else, s = -1; end
end

function a = wrap_angle(a)
    a = mod(a + pi, 2*pi) - pi;
end
