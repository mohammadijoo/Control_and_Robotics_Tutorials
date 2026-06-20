% Chapter16_Lesson1.m
% Sensing Dynamic Obstacles — KF Tracking + Gating (MATLAB teaching baseline)
%
% This script:
% 1) Simulates constant-velocity target(s)
% 2) Generates noisy position measurements + clutter
% 3) Runs a simple KF with chi-square gating
%
% Notes for Simulink:
% - Use a Discrete State-Space block for prediction (F, Q)
% - Use MATLAB Function blocks for update + gating
% - Feed detections from your perception subsystem (LiDAR clusters)

clear; clc; close all;

dt = 0.1;
T  = 120;

sigma_a = 0.7;
sigma_z = 0.35;

% State: [px; py; vx; vy]
F = [1 0 dt 0;
     0 1 0 dt;
     0 0 1 0;
     0 0 0 1];

q = sigma_a^2;
Q = q * [dt^4/4 0      dt^3/2 0;
         0      dt^4/4 0      dt^3/2;
         dt^3/2 0      dt^2   0;
         0      dt^3/2 0      dt^2];

H = [1 0 0 0;
     0 1 0 0];

R = (sigma_z^2) * eye(2);

% Chi-square gate (dof=2). Common values: pg=0.99 -> 9.210, pg=0.95 -> 5.991
pg = 0.99;
gamma = 9.210;

% Truth
x_true = [0; 0; 1.0; 0.6];

% Filter init
x = [0; 0; 0; 0];
P = diag([1, 1, 2, 2]);

Xtrue = zeros(4, T);
Xhat  = zeros(4, T);
Zmeas = zeros(2, T);
Accepted = false(1, T);

for k = 1:T
    % propagate truth
    x_true = F * x_true;
    Xtrue(:,k) = x_true;

    % measurement
    z = H * x_true + sigma_z * randn(2,1);
    Zmeas(:,k) = z;

    % predict
    x = F * x;
    P = F * P * F' + Q;

    % gate test
    y = z - H*x;
    S = H*P*H' + R;
    d2 = y' / S * y;

    if d2 <= gamma
        % update (Joseph form)
        K = P * H' / S;
        x = x + K*y;
        I = eye(4);
        P = (I - K*H) * P * (I - K*H)' + K*R*K';
        Accepted(k) = true;
    end

    Xhat(:,k) = x;
end

% Plot
figure; hold on; grid on;
plot(Xtrue(1,:), Xtrue(2,:), 'LineWidth', 2);
plot(Xhat(1,:),  Xhat(2,:),  '--', 'LineWidth', 1.5);
scatter(Zmeas(1,Accepted), Zmeas(2,Accepted), 18, 'filled', 'MarkerFaceAlpha', 0.35);
legend('Truth','KF estimate','Accepted measurements');
axis equal;
title('Dynamic obstacle sensing: KF tracking with chi-square gating');
xlabel('x [m]'); ylabel('y [m]');
