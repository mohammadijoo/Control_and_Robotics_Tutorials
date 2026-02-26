% Chapter16_Lesson3.m
% Socially Aware Navigation (sampling-based local velocity selection)
% Run: Chapter16_Lesson3
%
% Simulink note:
%   Put the function "choose_velocity" in a MATLAB Function block.
%   Inputs: p (2x1), goal (2x1), humansP (2xN), humansV (2xN)
%   Output: v_cmd (2x1)

function Chapter16_Lesson3()
clc; close all;

dt = 0.1;
T  = 26.0;
steps = round(T/dt);

p    = [0;0];
goal = [10;0];
v_max = 1.2;

humansP = [ 4.5  4.0  7.0;
            1.2 -1.4  0.3 ];
humansV = [ 0.35  0.35 -0.25;
           -0.05  0.08  0.02 ];

traj = zeros(2, steps);
trajH = zeros(2, steps, size(humansP,2));

for k = 1:steps
    traj(:,k) = p;
    trajH(:,k,:) = humansP;

    humansP = humansP + dt*humansV;

    e = goal - p;
    dist = norm(e);
    if dist < 0.15
        traj = traj(:,1:k);
        trajH = trajH(:,1:k,:);
        break;
    end
    v_pref = (e/(dist+1e-12)) * min(v_max, 0.8*dist);

    v_cmd = choose_velocity(p, v_pref, humansP, humansV, v_max, 3.0);
    p = p + dt*v_cmd;
end

figure; hold on; grid on; axis equal;
plot(traj(1,:), traj(2,:), 'LineWidth', 2);
for i=1:size(humansP,2)
    plot(squeeze(trajH(1,:,i)), squeeze(trajH(2,:,i)), '--', 'LineWidth', 1.5);
end
plot(goal(1), goal(2), '*', 'MarkerSize', 12, 'LineWidth', 2);
title('Socially aware navigation: sampling-based local velocity selection');
legend(['robot', arrayfun(@(i) sprintf('human %d',i), 1:size(humansP,2), 'UniformOutput', false)]);
end

function v_best = choose_velocity(p, v_pref, humansP, humansV, v_max, tau)
w_track  = 1.0;
w_social = 1.5;
w_clear  = 2.0;
Rcol = 0.55;

bestJ = inf;
v_best = [0;0];

V = sample_velocities(v_pref, v_max, 9, 21, deg2rad(80));

for k=1:size(V,2)
    v = V(:,k);
    if collision_within_tau(v, p, humansP, humansV, Rcol, tau)
        continue;
    end

    dt_eval = 0.6;
    p_next = p + dt_eval*v;

    C = anisotropic_gaussian_cost(p_next, humansP, humansV, 1.2, 0.6, 0.8);

    clear = 0.0;
    eps = 1e-3;
    for i=1:size(humansP,2)
        d = norm(p_next - humansP(:,i));
        clear = clear + 1.0/(d + eps);
    end

    J = w_track*sum((v - v_pref).^2) + w_social*C + w_clear*clear;
    if J < bestJ
        bestJ = J;
        v_best = v;
    end
end
end

function V = sample_velocities(v_pref, v_max, n_speed, n_angle, spread)
spref = norm(v_pref);
th0 = atan2(v_pref(2), v_pref(1));
if spref < 1e-8
    th0 = 0.0;
end
speeds = linspace(0, v_max, n_speed);
angles = linspace(-spread, spread, n_angle);

V = zeros(2, n_speed*n_angle);
idx = 1;
for s = speeds
    for a = angles
        th = th0 + a;
        V(:,idx) = [s*cos(th); s*sin(th)];
        idx = idx + 1;
    end
end
end

function tf = collision_within_tau(v_robot, p_robot, humansP, humansV, R, tau)
tf = false;
for i=1:size(humansP,2)
    p = humansP(:,i) - p_robot;
    v_rel = v_robot - humansV(:,i);
    vv = sum(v_rel.^2);
    if vv < 1e-12
        if norm(p) < R, tf = true; return; end
        continue;
    end
    t_star = - (p.'*v_rel) / vv;
    t_star = max(0, min(tau, t_star));
    d_min = norm(p - t_star*v_rel);
    if d_min < R, tf = true; return; end
end
end

function cost = anisotropic_gaussian_cost(x, humansP, humansV, sigma_front, sigma_side, sigma_back)
cost = 0.0;
for i=1:size(humansP,2)
    p = humansP(:,i);
    v = humansV(:,i);
    spd = norm(v);
    th = 0.0;
    if spd > 1e-6, th = atan2(v(2), v(1)); end
    R = [cos(th) -sin(th); sin(th) cos(th)];

    r = x - p;
    r_h = R.'*r;
    sx = sigma_front; if r_h(1) < 0, sx = sigma_back; end
    sy = sigma_side;

    q = (r_h(1)^2)/(sx^2) + (r_h(2)^2)/(sy^2);
    cost = cost + exp(-0.5*q);
end
end
