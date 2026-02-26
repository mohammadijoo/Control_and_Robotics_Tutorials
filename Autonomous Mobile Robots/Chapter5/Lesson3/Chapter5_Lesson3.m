% Chapter5_Lesson3.m
% Autonomous Mobile Robots (Control Engineering) — Chapter 5, Lesson 3
% Topic: Drift Sources and Bias Accumulation
%
% This script simulates 2D dead-reckoning and also (optionally) builds a tiny
% Simulink model to illustrate bias integration.
%
% MATLAB toolboxes used (optional):
%   - Simulink (for the model-building demo)
%
% Run:
%   Chapter5_Lesson3

clear; clc;

% -----------------------------
% Part A: Dead-reckoning simulation
% -----------------------------
T  = 60.0;
dt = 0.01;
N  = floor(T/dt);
t  = (0:N-1)'*dt;

% True parameters
r_true = 0.05;
b_true = 0.30;
ticks_per_rev = 2048;

% Estimator parameters (slightly wrong)
r_hat = r_true*1.005;
b_hat = b_true*0.995;

% True wheel radius mismatch
eps_r_L = +0.002;
eps_r_R = -0.002;
rL_true = r_true*(1+eps_r_L);
rR_true = r_true*(1+eps_r_R);

% Encoder
rad_per_tick = 2*pi/ticks_per_rev;
encoder_tick_noise_std = 0.2;

% Gyro
gyro_bias = 0.005;       % rad/s
gyro_noise_std = 0.002;  % rad/s

rng(7);

% Motion command
v_cmd = 0.6*ones(N,1);
w_cmd = 0.10*sin(2*pi*t/20.0);

% States: truth and estimates
x=zeros(N,1); y=zeros(N,1); th=zeros(N,1);
xw=zeros(N,1); yw=zeros(N,1); thw=zeros(N,1);
xg=zeros(N,1); yg=zeros(N,1); thg=zeros(N,1);
th_gyro = 0;

wrapPi = @(a) mod(a+pi,2*pi)-pi;

for k=2:N
    v = v_cmd(k-1); w = w_cmd(k-1);

    % True wheel angular velocities
    wR = (v + 0.5*b_true*w)/rR_true;
    wL = (v - 0.5*b_true*w)/rL_true;

    dphiR_true = wR*dt;
    dphiL_true = wL*dt;

    % Quantize to ticks + noise
    ticksR = dphiR_true/rad_per_tick;
    ticksL = dphiL_true/rad_per_tick;

    ticksR_meas = round(ticksR) + encoder_tick_noise_std*randn();
    ticksL_meas = round(ticksL) + encoder_tick_noise_std*randn();

    dphiR_meas = ticksR_meas*rad_per_tick;
    dphiL_meas = ticksL_meas*rad_per_tick;

    % Ground truth update
    dSR_true = rR_true*dphiR_true;
    dSL_true = rL_true*dphiL_true;
    dS_true  = 0.5*(dSR_true + dSL_true);
    dTh_true = (dSR_true - dSL_true)/b_true;

    th(k) = wrapPi(th(k-1) + dTh_true);
    x(k)  = x(k-1) + dS_true*cos(th(k-1) + 0.5*dTh_true);
    y(k)  = y(k-1) + dS_true*sin(th(k-1) + 0.5*dTh_true);

    % Wheel-only estimate
    dSR_hat = r_hat*dphiR_meas;
    dSL_hat = r_hat*dphiL_meas;
    dS_hat  = 0.5*(dSR_hat + dSL_hat);
    dTh_hat = (dSR_hat - dSL_hat)/b_hat;

    thw(k) = wrapPi(thw(k-1) + dTh_hat);
    xw(k)  = xw(k-1) + dS_hat*cos(thw(k-1) + 0.5*dTh_hat);
    yw(k)  = yw(k-1) + dS_hat*sin(thw(k-1) + 0.5*dTh_hat);

    % Gyro heading
    w_meas = w + gyro_bias + gyro_noise_std*randn();
    th_gyro = wrapPi(th_gyro + w_meas*dt);

    thg(k) = th_gyro;
    xg(k)  = xg(k-1) + dS_hat*cos(thg(k-1));
    yg(k)  = yg(k-1) + dS_hat*sin(thg(k-1));
end

e_w = sqrt((xw-x).^2 + (yw-y).^2);
e_g = sqrt((xg-x).^2 + (yg-y).^2);

fprintf('Final position error (wheel-only) [m]: %.4f\\n', e_w(end));
fprintf('Final position error (gyro-heading) [m]: %.4f\\n', e_g(end));
fprintf('Final heading error (wheel-only) [deg]: %.4f\\n', rad2deg(wrapPi(thw(end)-th(end))));
fprintf('Final heading error (gyro-heading) [deg]: %.4f\\n', rad2deg(wrapPi(thg(end)-th(end))));

figure; plot(x,y,'LineWidth',1.5); hold on;
plot(xw,yw,'LineWidth',1.2);
plot(xg,yg,'LineWidth',1.2);
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Dead-reckoning drift under bias and noise');
legend('truth','wheel-only','gyro-heading + wheel-distance');

figure; plot(t,e_w,'LineWidth',1.2); hold on;
plot(t,e_g,'LineWidth',1.2); grid on;
xlabel('time [s]'); ylabel('||position error|| [m]');
title('Error accumulation over time');
legend('wheel-only','gyro-heading');

% -----------------------------
% Part B (optional): Programmatic Simulink demo
% -----------------------------
% This small model integrates a constant gyro bias b_g [rad/s] to show that
% heading error grows linearly in time: theta_err(t)=b_g*t.
% If you do not have Simulink, comment out this section.

doSimulink = false; % set true to build and run the Simulink model
if doSimulink
    mdl = 'Chapter5_Lesson3_BiasAccumulation';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);

    add_block('simulink/Sources/Constant', [mdl '/gyro_bias'], 'Value', '0.005', 'Position', [30 40 90 70]);
    add_block('simulink/Continuous/Integrator', [mdl '/integrator_theta'], 'Position', [150 35 190 75]);
    add_block('simulink/Sinks/To Workspace', [mdl '/toWS'], 'VariableName', 'theta_err', 'SaveFormat', 'Array', 'Position', [260 40 330 70]);

    add_line(mdl, 'gyro_bias/1', 'integrator_theta/1');
    add_line(mdl, 'integrator_theta/1', 'toWS/1');

    set_param(mdl, 'StopTime', '60');
    sim(mdl);

    figure; plot(theta_err(:,1), theta_err(:,2)); grid on;
    xlabel('time [s]'); ylabel('theta\_err [rad]');
    title('Simulink demo: heading error accumulation from constant bias');
end
