% Chapter1_Lesson5.m
% Autonomous Mobile Robots — Chapter 1, Lesson 5
% Typical AMR Failure Modes (drift, slip, occlusion)
%
% This MATLAB script simulates a planar unicycle-like mobile robot and shows:
%   - Drift due to biased odometry
%   - Slip as attenuation + bursts
%   - Occlusion/outliers in a simple landmark measurement stream
%   - A lightweight linearized correction with gating
%
% Requirements:
%   - MATLAB base + plotting
% Optional:
%   - Simulink (for the optional programmatic model stub near the end)

clear; clc; close all;
rng(7);

T = 40.0;
dt = 0.02;
N = floor(T/dt);

landmark = [8.0; 6.0];

% states
x_true = [0;0;0];
x_hat  = [0;0;0];

% command
u_cmd = @(t) [0.6 + 0.15*sin(0.4*t);
              0.25*sin(0.25*t) + 0.20*sin(0.05*t)];

% drift parameters
b_v = 0.03; b_w = -0.015;
s_v = 1.03; s_w = 0.98;

% slip parameters
s_long_nom = 0.05; s_yaw_nom = 0.03;
slip_burst_prob = 0.02;
slip_burst_mag  = 0.35;

% noise
odo_sigma_v = 0.02; odo_sigma_w = 0.02;
meas_sigma_r = 0.08;
meas_sigma_b = deg2rad(1.5);

% correction
R = diag([meas_sigma_r^2, meas_sigma_b^2]);
P = diag([0.2^2, 0.2^2, deg2rad(5)^2]);
gate = 9.21;

meas_period = 0.5;
next_meas_t = 0.0;
p_occlude = 0.35;
p_outlier = 0.08;

% logs
t_log = (0:N-1)'*dt;
X_true = zeros(N,3);
X_hat  = zeros(N,3);
Z      = nan(N,2);
Z_used = false(N,1);

for k=1:N
    t = t_log(k);
    u = u_cmd(t);

    % slip
    burst = rand() < slip_burst_prob;
    s_long = min(0.9, max(0.0, s_long_nom + (burst*slip_burst_mag)));
    s_yaw  = min(0.9, max(0.0, s_yaw_nom  + (burst*0.5*slip_burst_mag)));

    u_true = [(1-s_long)*u(1); (1-s_yaw)*u(2)];
    x_true = unicycle_step(x_true, u_true, dt);

    % odometry (biased + noisy)
    u_odo = [s_v*u(1) + b_v + odo_sigma_v*randn();
             s_w*u(2) + b_w + odo_sigma_w*randn()];
    x_hat = unicycle_step(x_hat, u_odo, dt);

    % measurement
    if t + 1e-12 >= next_meas_t
        next_meas_t = next_meas_t + meas_period;

        if rand() >= p_occlude
            z = landmark_meas(x_true, landmark);
            z = z + [meas_sigma_r*randn(); meas_sigma_b*randn()];
            z(2) = wrap_to_pi(z(2));

            if rand() < p_outlier
                z(1) = z(1) + 2.0*randn();
                z(2) = wrap_to_pi(z(2) + deg2rad(25.0)*randn());
            end

            Z(k,:) = z';

            zhat = landmark_meas(x_hat, landmark);
            r = [z(1)-zhat(1); wrap_to_pi(z(2)-zhat(2))];

            H = jacobian_landmark(x_hat, landmark);
            S = H*P*H' + R;
            d2 = r'*(S\r);

            if d2 < gate
                K = P*H'/S;
                dx = K*r;
                x_hat = x_hat + dx;
                x_hat(3) = wrap_to_pi(x_hat(3));
                P = (eye(3)-K*H)*P*(eye(3)-K*H)' + K*R*K';
                Z_used(k) = true;
            end
        end
    end

    % mild inflation
    P = P + diag([1e-4, 1e-4, 1e-6]);

    X_true(k,:) = x_true';
    X_hat(k,:)  = x_hat';
end

% plots
figure;
plot(X_true(:,1), X_true(:,2), 'LineWidth', 1.5); hold on;
plot(X_hat(:,1),  X_hat(:,2),  'LineWidth', 1.5);
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Trajectory: true vs estimated (drift + slip + occlusion)');
legend('true','estimated');

e = X_hat - X_true;
e(:,3) = arrayfun(@wrap_to_pi, e(:,3));

figure;
plot(t_log, e(:,1), 'LineWidth', 1.2); hold on;
plot(t_log, e(:,2), 'LineWidth', 1.2);
plot(t_log, e(:,3), 'LineWidth', 1.2);
grid on;
xlabel('time [s]'); ylabel('error');
title('Estimation error');
legend('e_x','e_y','e_\theta');

idx = find(~isnan(Z(:,1)));
if ~isempty(idx)
    figure;
    plot(t_log(idx), Z(idx,1), '.', 'MarkerSize', 10); hold on;
    plot(t_log(idx(Z_used(idx))), Z(idx(Z_used(idx)),1), 'o', 'MarkerSize', 7);
    grid on;
    xlabel('time [s]'); ylabel('range [m]');
    title('Landmark measurement stream (used after gating)');
    legend('range meas','used');
end

% Optional: programmatic Simulink stub (uncomment if Simulink is available)
% build_simulink_stub();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x2 = unicycle_step(x, u, dt)
    px = x(1); py = x(2); th = x(3);
    v = u(1); w = u(2);
    x2 = [px + dt*v*cos(th);
          py + dt*v*sin(th);
          wrap_to_pi(th + dt*w)];
end

function z = landmark_meas(x, lm)
    dx = lm(1) - x(1);
    dy = lm(2) - x(2);
    r = sqrt(dx^2 + dy^2);
    b = wrap_to_pi(atan2(dy, dx) - x(3));
    z = [r; b];
end

function H = jacobian_landmark(x, lm)
    dx = lm(1) - x(1);
    dy = lm(2) - x(2);
    q = dx^2 + dy^2;
    q = max(q, 1e-12);
    r = sqrt(q);
    r = max(r, 1e-12);

    H = zeros(2,3);
    H(1,1) = -dx/r;
    H(1,2) = -dy/r;

    H(2,1) =  dy/q;
    H(2,2) = -dx/q;
    H(2,3) = -1.0;
end

function a = wrap_to_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end

function build_simulink_stub()
    % Build a minimal Simulink model skeleton for a planar integrator chain.
    % This is a pedagogical stub (not a full navigation pipeline model).
    mdl = 'Chapter1_Lesson5_SimulinkStub';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    open_system(mdl);

    add_block('simulink/Sources/Constant', [mdl '/v'], 'Value', '0.5');
    add_block('simulink/Sources/Constant', [mdl '/w'], 'Value', '0.1');
    add_block('simulink/Continuous/Integrator', [mdl '/Int_x']);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_y']);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_th']);

    % You would add Trigonometric Function blocks for cos(th), sin(th),
    % multiply by v, and integrate. This stub intentionally stays minimal.
    set_param(mdl, 'StopTime', '10');
    save_system(mdl);
    disp(['Built Simulink stub: ' mdl]);
end
