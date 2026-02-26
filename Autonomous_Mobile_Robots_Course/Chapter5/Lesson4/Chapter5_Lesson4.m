% Chapter5_Lesson4.m
% Autonomous Mobile Robots — Chapter 5 Lesson 4: Practical Odometry Filtering
%
% Demonstrates:
%  - Hampel outlier suppression (median + MAD)
%  - First-order low-pass filtering (IIR)
%  - Complementary heading fusion: wheel heading (low-freq) + gyro integration (high-freq)
%  - Planar pose integration
%  - Programmatic construction of a simple Simulink model implementing the same filter logic
%
% Toolboxes (optional but useful):
%  - Robotics System Toolbox (for ROS message types, not required here)
%  - Simulink (required to build the model)
%
% Run:
%   Chapter5_Lesson4
%
% Output:
%   - Plots
%   - A Simulink model file: Chapter5_Lesson4_OdometryFilter.slx (if Simulink is available)

clear; clc; close all;

dt = 0.01;
T = 20.0;
N = floor(T/dt);
t = (0:N-1)' * dt;

% --- Truth signals
vTrue = zeros(N,1);
wTrue = zeros(N,1);
for k = 1:N
    if t(k) >= 1.0 && t(k) < 6.0
        vTrue(k) = 1.1; wTrue(k) = 0.0;
    elseif t(k) >= 6.0 && t(k) < 12.0
        vTrue(k) = 0.8; wTrue(k) = 0.35;
    elseif t(k) >= 12.0 && t(k) < 16.0
        vTrue(k) = 0.0; wTrue(k) = -0.6;
    elseif t(k) >= 16.0 && t(k) < 20.0
        vTrue(k) = 1.0; wTrue(k) = 0.15;
    end
end

thetaTrue = zeros(N,1);
for k = 2:N
    thetaTrue(k) = wrapToPi(thetaTrue(k-1) + dt*wTrue(k));
end

% --- Measurements
rng(2);
vW = vTrue + 0.05*randn(N,1);
wW = wTrue + 0.08*randn(N,1);

qv = 0.02; qw = 0.02;
vW = round(vW/qv)*qv;
wW = round(wW/qw)*qw;

spikeIdx = randperm(N, 12);
vW(spikeIdx) = vW(spikeIdx) + 0.8*randn(12,1);
wW(spikeIdx) = wW(spikeIdx) + 2.0*randn(12,1);

thetaW = zeros(N,1);
for k = 2:N
    thetaW(k) = wrapToPi(thetaW(k-1) + dt*wW(k));
end

bias = zeros(N,1);
bias(1) = 0.03;
for k = 2:N
    bias(k) = bias(k-1) + 0.0005*randn();
end
wG = wTrue + bias + 0.05*randn(N,1);

% --- Hampel filter
vWh = hampel1d(vW, 11, 3.0);
wWh = hampel1d(wW, 11, 3.0);

% --- Low-pass coefficients (exact discretization)
vFc = 5.0;  wFc = 8.0;  thFc = 0.7;
vAlpha = exp(-dt / (1/(2*pi*vFc)));
wAlpha = exp(-dt / (1/(2*pi*wFc)));
thAlpha = exp(-dt / (1/(2*pi*thFc)));

vMax = 2.0; wMax = 3.0; slipGate = 1.2;

% --- Filtering loop
xF = zeros(N,1); yF = zeros(N,1); thF = zeros(N,1);
vF = zeros(N,1); wF = zeros(N,1);

for k = 1:N
    v = min(max(vWh(k), -vMax), vMax);
    ww = min(max(wWh(k), -wMax), wMax);
    wg = min(max(wG(k), -wMax), wMax);

    if k == 1
        vF(k) = v; wF(k) = ww; thF(k) = thetaW(k);
    else
        vF(k) = vAlpha*vF(k-1) + (1-vAlpha)*v;
        wF(k) = wAlpha*wF(k-1) + (1-wAlpha)*ww;

        resid = abs(wg - ww);
        alphaEff = thAlpha;
        if resid > slipGate
            alphaEff = min(0.995, thAlpha + 0.15);
        end

        pred = wrapToPi(thF(k-1) + dt*wg);
        thF(k) = wrapToPi(alphaEff*pred + (1-alphaEff)*thetaW(k));
    end

    if k == 1
        xF(k) = 0; yF(k) = 0;
    else
        xF(k) = xF(k-1) + dt*vF(k)*cos(thF(k));
        yF(k) = yF(k-1) + dt*vF(k)*sin(thF(k));
    end
end

% --- Truth position
xTrue = zeros(N,1); yTrue = zeros(N,1);
for k = 2:N
    xTrue(k) = xTrue(k-1) + dt*vTrue(k)*cos(thetaTrue(k));
    yTrue(k) = yTrue(k-1) + dt*vTrue(k)*sin(thetaTrue(k));
end

figure; plot(t, vTrue, 'LineWidth', 1.2); hold on;
plot(t, vW, 'LineWidth', 0.7);
plot(t, vWh, 'LineWidth', 1.0);
legend('v true','v wheel raw','v wheel Hampel'); xlabel('t [s]'); ylabel('v [m/s]');
title('Linear velocity filtering');

figure; plot(t, wTrue, 'LineWidth', 1.2); hold on;
plot(t, wW, 'LineWidth', 0.7);
plot(t, wWh, 'LineWidth', 1.0);
plot(t, wG, 'LineWidth', 0.9);
legend('w true','w wheel raw','w wheel Hampel','w gyro'); xlabel('t [s]'); ylabel('w [rad/s]');
title('Yaw-rate streams');

figure; plot(xTrue, yTrue, 'LineWidth', 1.2); hold on;
plot(xF, yF, 'LineWidth', 1.2); axis equal; grid on;
legend('truth','filtered odometry'); xlabel('x [m]'); ylabel('y [m]');
title('Trajectory comparison');

figure; eTh = wrapToPi(thF - thetaTrue);
plot(t, eTh, 'LineWidth', 1.2); xlabel('t [s]'); ylabel('heading error [rad]');
title('Heading error after complementary fusion');

% --- Build Simulink model programmatically (optional)
try
    buildSimulinkModel();
    disp('Built Simulink model: Chapter5_Lesson4_OdometryFilter.slx');
catch ME
    disp('Simulink model build skipped (Simulink may be unavailable):');
    disp(ME.message);
end

% ---------- Local functions ----------
function y = wrapToPi(a)
    y = mod(a + pi, 2*pi) - pi;
end

function y = hampel1d(x, window, nSigmas)
    if mod(window,2) == 0
        error('window must be odd');
    end
    k = floor(window/2);
    y = x;
    for i = (k+1):(length(x)-k)
        w = x((i-k):(i+k));
        med = median(w);
        MAD = median(abs(w-med)) + 1e-12;
        sigma = 1.4826*MAD;
        if abs(x(i)-med) > nSigmas*sigma
            y(i) = med;
        end
    end
end

function buildSimulinkModel()
    mdl = 'Chapter5_Lesson4_OdometryFilter';
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    new_system(mdl);
    open_system(mdl);

    % Blocks: In1 (v), In1 (w_g), In1 (theta_w), Unit Delay, Gain, Sum, etc.
    % This is a compact educational model (not a full ROS pipeline).

    add_block('simulink/Sources/In1', [mdl '/v_w'], 'Position', [30 50 60 70]);
    add_block('simulink/Sources/In1', [mdl '/omega_g'], 'Position', [30 120 60 140]);
    add_block('simulink/Sources/In1', [mdl '/theta_w'], 'Position', [30 190 60 210]);

    % Discrete low-pass for v: y = alpha*z^-1*y + (1-alpha)*x
    add_block('simulink/Discrete/Unit Delay', [mdl '/z1_v'], 'Position', [150 45 190 75]);
    add_block('simulink/Math Operations/Gain', [mdl '/alpha_v'], 'Gain', '0.9', 'Position', [230 45 280 75]);
    add_block('simulink/Math Operations/Gain', [mdl '/one_minus_alpha_v'], 'Gain', '0.1', 'Position', [230 90 280 120]);
    add_block('simulink/Math Operations/Sum', [mdl '/sum_v'], 'Inputs', '++', 'Position', [320 60 350 100]);

    add_line(mdl, 'v_w/1', 'one_minus_alpha_v/1');
    add_line(mdl, 'z1_v/1', 'alpha_v/1');
    add_line(mdl, 'alpha_v/1', 'sum_v/1');
    add_line(mdl, 'one_minus_alpha_v/1', 'sum_v/2');
    add_line(mdl, 'sum_v/1', 'z1_v/1');

    add_block('simulink/Sinks/Out1', [mdl '/v_f'], 'Position', [390 70 420 90]);
    add_line(mdl, 'sum_v/1', 'v_f/1');

    % Complementary heading: theta = alpha*(z^-1*theta + dt*omega_g) + (1-alpha)*theta_w
    add_block('simulink/Discrete/Unit Delay', [mdl '/z1_theta'], 'Position', [150 160 190 190]);
    add_block('simulink/Math Operations/Gain', [mdl '/dt'], 'Gain', '0.01', 'Position', [150 110 190 140]);
    add_block('simulink/Math Operations/Sum', [mdl '/pred'], 'Inputs', '++', 'Position', [230 135 260 175]);

    add_block('simulink/Math Operations/Gain', [mdl '/alpha_theta'], 'Gain', '0.95', 'Position', [300 135 350 165]);
    add_block('simulink/Math Operations/Gain', [mdl '/one_minus_alpha_theta'], 'Gain', '0.05', 'Position', [300 195 350 225]);
    add_block('simulink/Math Operations/Sum', [mdl '/sum_theta'], 'Inputs', '++', 'Position', [390 160 420 200]);
    add_block('simulink/Sinks/Out1', [mdl '/theta_f'], 'Position', [460 170 490 190]);

    add_line(mdl, 'omega_g/1', 'dt/1');
    add_line(mdl, 'dt/1', 'pred/2');
    add_line(mdl, 'z1_theta/1', 'pred/1');

    add_line(mdl, 'pred/1', 'alpha_theta/1');
    add_line(mdl, 'theta_w/1', 'one_minus_alpha_theta/1');
    add_line(mdl, 'alpha_theta/1', 'sum_theta/1');
    add_line(mdl, 'one_minus_alpha_theta/1', 'sum_theta/2');
    add_line(mdl, 'sum_theta/1', 'z1_theta/1');
    add_line(mdl, 'sum_theta/1', 'theta_f/1');

    set_param(mdl, 'StopTime', '20');
    save_system(mdl);
    close_system(mdl, 1);
end
