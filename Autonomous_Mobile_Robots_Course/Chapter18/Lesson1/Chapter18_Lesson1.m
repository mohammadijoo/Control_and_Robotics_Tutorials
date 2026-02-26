% Chapter18_Lesson1.m
% Autonomous Mobile Robots (Control Engineering) — Chapter 18, Lesson 1
% GPS/RTK Integration in Navigation (didactic EKF fusion example)
%
% This MATLAB script implements an EKF that fuses wheel speed, gyro yaw-rate
% (with a bias state), and GNSS/RTK position in a local ENU frame.
%
% State: x = [px; py; yaw; v; bg]
% Inputs: v_meas, omega_meas
% Measurement: z_gnss = [px; py]
%
% NOTE: This is a teaching implementation; for production consider MATLAB
% Navigation Toolbox / Sensor Fusion and Tracking Toolbox.

clear; clc;

% -----------------------------
% Configuration
% -----------------------------
cfg.q_xy  = 0.05;
cfg.q_yaw = 0.01;
cfg.q_v   = 0.5;
cfg.q_bg  = 1e-4;

cfg.r_v = 0.2^2;

cfg.r_gps_fix = 1.5^2;
cfg.r_gps_rtk = 0.02^2;

cfg.gate_chi2_dof2 = 9.2103; % chi2inv(0.99,2)

wrapToPi = @(a) mod(a+pi, 2*pi) - pi;

% -----------------------------
% EKF initialization
% -----------------------------
x = [0; 0; 0; 1.0; 0]; % [px, py, yaw, v, bg]
P = diag([10, 10, (20*pi/180)^2, 2^2, (5*pi/180)^2]);

% -----------------------------
% Simulation setup (same as Python/C++)
% -----------------------------
dt = 0.05; T = 60; n = floor(T/dt);

v_true = 1.2;
omega_true = 0.07;
bg_true = 0.02;

px = 0; py = 0; yaw = 0;

rng(42);

log = zeros(n, 8); % t, true_x, true_y, true_yaw, est_x, est_y, est_yaw, gnss_ok

for k = 1:n
    t = (k-1)*dt;

    % Truth
    yaw = wrapToPi(yaw + omega_true*dt);
    px = px + v_true*cos(yaw)*dt;
    py = py + v_true*sin(yaw)*dt;

    % Measurements
    v_meas = v_true + 0.05*randn();
    omega_meas = omega_true + bg_true + 0.01*randn();

    % -----------------------------
    % Predict
    % -----------------------------
    omega = omega_meas - x(5);
    x_pred = x;
    x_pred(3) = wrapToPi(x(3) + omega*dt);
    x_pred(1) = x(1) + x(4)*cos(x_pred(3))*dt;
    x_pred(2) = x(2) + x(4)*sin(x_pred(3))*dt;
    % v random-walk, bg random-walk: unchanged

    F = eye(5);
    F(1,3) = -x(4)*sin(x_pred(3))*dt;
    F(1,4) =  cos(x_pred(3))*dt;
    F(2,3) =  x(4)*cos(x_pred(3))*dt;
    F(2,4) =  sin(x_pred(3))*dt;
    F(3,5) = -dt;

    Q = diag([cfg.q_xy*dt, cfg.q_xy*dt, cfg.q_yaw*dt, cfg.q_v*dt, cfg.q_bg*dt]);

    x = x_pred;
    P = F*P*F' + Q;

    % Optional: wheel speed pseudo-measurement update for v
    H_v = zeros(1,5); H_v(4) = 1;
    z_v = v_meas; h_v = x(4);
    y_v = z_v - h_v;
    S_v = H_v*P*H_v' + cfg.r_v;
    K_v = P*H_v'/S_v;
    x = x + K_v*y_v;
    P = (eye(5) - K_v*H_v)*P;

    % -----------------------------
    % GNSS update at 5 Hz
    % -----------------------------
    gnss_ok = 0;
    if mod(k-1, round(0.2/dt)) == 0
        fix_q = 4;
        if mod(k-1, 200) == 0
            fix_q = 5;
        end
        if fix_q == 4
            r_pos = cfg.r_gps_rtk;
        else
            r_pos = 0.2^2;
        end

        z = [px; py] + sqrt(r_pos)*randn(2,1);
        H = [1 0 0 0 0;
             0 1 0 0 0];
        h = x(1:2);
        R = diag([r_pos, r_pos]);

        innov = z - h;
        S = H*P*H' + R;
        d2 = innov'*(S\innov);
        if d2 <= cfg.gate_chi2_dof2
            K = P*H'/S;
            x = x + K*innov;
            x(3) = wrapToPi(x(3));
            P = (eye(5) - K*H)*P;
            gnss_ok = 1;
        end
    end

    log(k,:) = [t, px, py, yaw, x(1), x(2), x(3), gnss_ok];
end

disp('t true_x true_y true_yaw est_x est_y est_yaw gnss_ok (tail)');
disp(log(end-9:end,:));

% -----------------------------
% Simulink hint (programmatic skeleton)
% -----------------------------
% You can construct a simple fusion diagram with:
%  - "Discrete-Time Integrator" for state propagation
%  - "MATLAB Function" block for EKF step (predict+update)
%  - "From Workspace" for simulated wheel/gyro/GNSS streams
% Example (minimal):
%   mdl = 'Chapter18_Lesson1_EKF';
%   new_system(mdl); open_system(mdl);
%   add_block('simulink/Sources/From Workspace',[mdl '/wheel_v']);
%   add_block('simulink/Sources/From Workspace',[mdl '/gyro_w']);
%   add_block('simulink/Sources/From Workspace',[mdl '/gnss_xy']);
%   add_block('simulink/User-Defined Functions/MATLAB Function',[mdl '/EKF']);
% Then wire the blocks and implement the EKF equations inside the MATLAB Function.
