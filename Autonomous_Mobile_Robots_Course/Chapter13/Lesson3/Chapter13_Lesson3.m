% Chapter13_Lesson3.m
% Visual–Inertial Fusion Pipelines (AMR Focus)
%
% Minimal planar EKF fusion: IMU propagation + visual pose update.
% This is a pedagogical baseline, not a production VIO.

clear; clc;

dt   = 0.01;
T    = 20.0;
voDt = 0.1;

rng(7);
N = floor(T/dt) + 1;
t = linspace(0, T, N)';

px = zeros(N,1); py = zeros(N,1);
vx = zeros(N,1); vy = zeros(N,1);
th = zeros(N,1);

speed   = 1.0 + 0.2*sin(0.3*t);
yawRate = 0.25*sin(0.2*t);

for k=2:N
    th(k) = wrapToPi(th(k-1) + yawRate(k-1)*dt);
    vx(k) = speed(k)*cos(th(k));
    vy(k) = speed(k)*sin(th(k));
    px(k) = px(k-1) + vx(k-1)*dt;
    py(k) = py(k-1) + vy(k-1)*dt;
end

axw = gradient(vx, dt);
ayw = gradient(vy, dt);
g = [0; -9.81];

b_a = [0.08; -0.05];
b_g = 0.01;
sigma_a = 0.12;
sigma_g = 0.015;

imu = zeros(N,3); % [axm, aym, wzm]
for k=1:N
    R = [cos(th(k)) -sin(th(k)); sin(th(k)) cos(th(k))];
    a_w = [axw(k); ayw(k)];
    a_b = R'*(a_w - g);
    imu(k,1:2) = (a_b + b_a + sigma_a*randn(2,1))';
    imu(k,3)   = yawRate(k) + b_g + sigma_g*randn();
end

step = floor(voDt/dt);
idx = 1:step:N;
vo_t = t(idx);
sigma_p = 0.05;
sigma_th = 0.02;
vo = [px(idx) + sigma_p*randn(numel(idx),1), ...
      py(idx) + sigma_p*randn(numel(idx),1), ...
      wrapToPi(th(idx) + sigma_th*randn(numel(idx),1))];

% EKF init: x=[px py vx vy th bax bay bg]'
x = zeros(8,1);
x(1) = vo(1,1);
x(2) = vo(1,2);
x(5) = vo(1,3);

P = diag([0.5, 0.5, 0.2, 0.2, 0.3, 0.2, 0.2, 0.1].^2);

Qc = zeros(8,8);
Qc(3,3) = 0.4^2;
Qc(4,4) = 0.4^2;
Qc(5,5) = 0.15^2;
Qc(6,6) = 0.01^2;
Qc(7,7) = 0.01^2;
Qc(8,8) = 0.002^2;

Rz = diag([0.05, 0.05, 0.02].^2);

est = zeros(N,8);
vo_ptr = 1;

for k=1:N
    u = imu(k,:)';
    [x, P] = imu_propagate_planar(x, P, u, Qc, dt);

    if vo_ptr <= size(vo,1) && abs(t(k) - vo_t(vo_ptr)) < 0.5*dt
        z = vo(vo_ptr,:)';
        [x, P] = ekf_update_pose_planar(x, P, z, Rz);
        vo_ptr = vo_ptr + 1;
    end

    est(k,:) = x';
end

final_err = [est(end,1)-px(end), est(end,2)-py(end), wrapToPi(est(end,5)-th(end))];
fprintf('Final error [m, m, rad]: %.3f %.3f %.3f\n', final_err(1), final_err(2), final_err(3));

figure; plot(px,py,'LineWidth',1.2); hold on;
plot(est(:,1), est(:,2),'LineWidth',1.2);
axis equal; grid on; legend('truth','EKF');
title('Planar VIO fusion: trajectory');

% ===============================
% Local functions
% ===============================
function [x_new, P_new] = imu_propagate_planar(x, P, u, Qc, dt)
    px = x(1); py = x(2); vx = x(3); vy = x(4); th = x(5);
    bax = x(6); bay = x(7); bg = x(8);

    axm = u(1); aym = u(2); wzm = u(3);
    g = [0; -9.81];

    a_b = [axm - bax; aym - bay];
    w = wzm - bg;

    th_new = wrapToPi(th + w*dt);
    R = [cos(th) -sin(th); sin(th) cos(th)];
    a_w = R*a_b + g;

    vx_new = vx + a_w(1)*dt;
    vy_new = vy + a_w(2)*dt;
    px_new = px + vx*dt + 0.5*a_w(1)*dt*dt;
    py_new = py + vy*dt + 0.5*a_w(2)*dt*dt;

    x_new = [px_new; py_new; vx_new; vy_new; th_new; bax; bay; bg];

    F = eye(8);
    F(1,3) = dt;
    F(2,4) = dt;

    dR_dth = [-sin(th) -cos(th); cos(th) -sin(th)];
    da_w_dth = dR_dth * a_b;

    F(3,5) = da_w_dth(1)*dt;
    F(4,5) = da_w_dth(2)*dt;

    F(3,6) = -R(1,1)*dt; F(3,7) = -R(1,2)*dt;
    F(4,6) = -R(2,1)*dt; F(4,7) = -R(2,2)*dt;

    F(1,5) = da_w_dth(1)*0.5*dt*dt;
    F(2,5) = da_w_dth(2)*0.5*dt*dt;

    F(1,6) = -R(1,1)*0.5*dt*dt; F(1,7) = -R(1,2)*0.5*dt*dt;
    F(2,6) = -R(2,1)*0.5*dt*dt; F(2,7) = -R(2,2)*0.5*dt*dt;

    F(5,8) = -dt;

    Qd = Qc * dt;
    P_new = F*P*F' + Qd;
end

function [x_upd, P_upd] = ekf_update_pose_planar(x, P, z, Rz)
    H = zeros(3,8);
    H(1,1) = 1;
    H(2,2) = 1;
    H(3,5) = 1;

    zhat = [x(1); x(2); x(5)];
    y = z - zhat;
    y(3) = wrapToPi(y(3));

    S = H*P*H' + Rz;
    K = P*H'/S;

    x_upd = x + K*y;
    x_upd(5) = wrapToPi(x_upd(5));

    I = eye(8);
    P_upd = (I - K*H)*P*(I - K*H)' + K*Rz*K';
end
