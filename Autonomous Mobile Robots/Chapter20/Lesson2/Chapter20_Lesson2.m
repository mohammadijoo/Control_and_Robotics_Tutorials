% Chapter20_Lesson2.m
% Sensor Suite Selection (given hardware) - MATLAB / Simulink support script
% This script compares candidate suites using covariance propagation and asynchronous updates.
% It also prints parameters that can be copied into Simulink blocks (sample times, R matrices).

clear; clc;

% State x = [x; y; theta; v; b_g]
n = 5;
dt = 0.01;           % base simulation step (100 Hz)
T  = 30.0;           % seconds
N  = round(T / dt);

% Constant-velocity planar linearized model (local frame approximation)
F = eye(n);
F(1,4) = dt;         % x <- x + v*dt
F(3,5) = -dt;        % theta affected by gyro bias (proxy)
Q = diag([0.03, 0.03, 0.01, 0.05, 0.002]) * dt;

% Candidate sensors (linearized H, R, and update periods)
sensors = struct([]);

sensors(1).name = 'wheel_encoder';
sensors(1).H = [0 0 0 1 0; 0 0 1 0 1];
sensors(1).R = diag([0.03^2, 0.01^2]);
sensors(1).Ts = 1/50;

sensors(2).name = 'imu';
sensors(2).H = [0 0 1 0 1; 0 0 0 1 0];
sensors(2).R = diag([0.015^2, 0.01^2]);
sensors(2).Ts = 1/200;

sensors(3).name = '2d_lidar';
sensors(3).H = [1 0 0 0 0; 0 1 0 0 0; 0 0 1 0 0];
sensors(3).R = diag([0.05^2, 0.05^2, 0.02^2]);
sensors(3).Ts = 1/10;

sensors(4).name = 'mono_camera';
sensors(4).H = [1 0 0 0 0; 0 1 0 0 0; 0 0 1 0 0];
sensors(4).R = diag([0.08^2, 0.08^2, 0.03^2]);
sensors(4).Ts = 1/30;

sensors(5).name = 'gnss_rtk';
sensors(5).H = [1 0 0 0 0; 0 1 0 0 0];
sensors(5).R = diag([0.03^2, 0.03^2]);
sensors(5).Ts = 1/5;

% Two example suites (given hardware, different choices)
suiteA = [1 2 3 5]; % wheel + imu + lidar + gnss
suiteB = [1 2 4 5]; % wheel + imu + camera + gnss

P0 = diag([1.5, 1.5, 0.5, 0.8, 0.2]);

[traceHistA, PfinalA] = run_suite(F, Q, sensors, suiteA, P0, dt, N);
[traceHistB, PfinalB] = run_suite(F, Q, sensors, suiteB, P0, dt, N);

fprintf('Final trace(P) suiteA = %.4f\n', trace(PfinalA));
fprintf('Final trace(P) suiteB = %.4f\n', trace(PfinalB));

% Plot covariance trace evolution
figure;
plot((0:N-1)*dt, traceHistA, 'LineWidth', 1.5); hold on;
plot((0:N-1)*dt, traceHistB, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('trace(P)');
legend('Suite A: enc+imu+lidar+gnss', 'Suite B: enc+imu+camera+gnss', 'Location', 'northeast');
title('Covariance Trace Comparison for Candidate Sensor Suites');
grid on;

% Print Simulink-friendly configuration
fprintf('\nSimulink block parameters (copy into Rate Transition / Sensor blocks):\n');
for i = 1:numel(sensors)
    fprintf('  %s: Ts = %.4f s, R = %s\n', sensors(i).name, sensors(i).Ts, mat2str(diag(sensors(i).R)'));
end

% -------------------------
function [traceHist, P] = run_suite(F, Q, sensors, suite, P0, dt, N)
    n = size(F,1);
    P = P0;
    traceHist = zeros(N,1);

    % Next measurement times
    nextT = inf(1, numel(sensors));
    for k = suite
        nextT(k) = 0.0;
    end

    t = 0.0;
    for i = 1:N
        % Time update
        P = F * P * F' + Q;

        % Measurement updates (asynchronous)
        for k = suite
            if t + 1e-12 >= nextT(k)
                H = sensors(k).H;
                R = sensors(k).R;
                S = H * P * H' + R;
                K = P * H' / S;
                P = (eye(n) - K * H) * P * (eye(n) - K * H)' + K * R * K'; % Joseph form
                nextT(k) = nextT(k) + sensors(k).Ts;
            end
        end

        traceHist(i) = trace(P);
        t = t + dt;
    end
end
