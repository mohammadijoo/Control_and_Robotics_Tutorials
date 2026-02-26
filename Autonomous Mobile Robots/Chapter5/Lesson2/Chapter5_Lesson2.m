% Chapter 5 — Odometry and Dead Reckoning
% Lesson 2: IMU Integration for Ground Robots
%
% Filename: Chapter5_Lesson2.m
%
% This file contains:
%  (1) A MATLAB implementation of deterministic IMU strapdown integration.
%  (2) A script that programmatically builds a minimal Simulink model that
%      runs the same integration inside a MATLAB Function block.
%
% Notes:
%  - This is intentionally "probability-free" (no Kalman filters), consistent
%    with Chapter 5 (dead reckoning).
%  - Requires only base MATLAB; Simulink part requires Simulink.

clear; clc;

%% Part A: MATLAB strapdown integration demo (synthetic turn)
N = 2000; dt = 0.01;
t = (0:N-1)' * dt;

v0 = 1.2; w0 = 0.20; g = 9.80665;

yaw_true = w0*t;
x_true = (v0/w0)*sin(yaw_true);
y_true = (v0/w0)*(1 - cos(yaw_true));

ax_nav = v0*w0*cos(yaw_true);
ay_nav = v0*w0*sin(yaw_true);
a_nav = [ax_nav, ay_nav, zeros(N,1)];

% specific force in body: f_b = R_bn (a - g)
f_b = zeros(N,3);
for k=1:N
    c = cos(yaw_true(k)); s = sin(yaw_true(k));
    R_bn = [ c, s, 0;
            -s, c, 0;
             0, 0, 1];
    f_b(k,:) = (R_bn * (a_nav(k,:)' - [0;0;-g]))';
end

gyro = [zeros(N,2), w0*ones(N,1)];
accel = f_b;

% Inject bias/noise
rng(1);
b_g = [0,0,0.01];      % rad/s
b_a = [0.05,-0.02,0];  % m/s^2
gyro_m  = gyro  + b_g + 0.002*randn(N,3);
accel_m = accel + b_a + 0.05*randn(N,3);

% Wheel yaw for complementary fusion (demo)
wheel_yaw = yaw_true + 0.01*randn(N,1) + 0.002*t;

params.g = g;
params.enforce_planar = true;
params.fuse_yaw_with_wheel = true;
params.tau_yaw = 2.0;

[p_est, v_est, yaw_est] = strapdown_integrate_matlab(t, gyro_m, accel_m, wheel_yaw, params);

e = [p_est(end,1) - x_true(end), p_est(end,2) - y_true(end)];
fprintf('Final position error [m] (MATLAB, fused yaw): (%.3f, %.3f)\\n', e(1), e(2));
fprintf('Final yaw error [deg]: %.3f\\n', rad2deg(wrapToPi(yaw_est(end) - yaw_true(end))));

%% Part B: Build a minimal Simulink model programmatically (optional)
% The model uses a single MATLAB Function block with persistent state that
% updates at fixed sample time Ts. This is a convenient way to represent
% strapdown integration without a large block diagram.
build_simulink_model = true;
if build_simulink_model
    modelName = 'Chapter5_Lesson2_Simulink';
    Ts = dt;
    try
        if bdIsLoaded(modelName), close_system(modelName, 0); end
        if exist([modelName '.slx'],'file'), delete([modelName '.slx']); end

        new_system(modelName);
        open_system(modelName);

        % Sources (From Workspace)
        add_block('simulink/Sources/From Workspace', [modelName '/gyro'], 'Position',[30 40 150 70]);
        add_block('simulink/Sources/From Workspace', [modelName '/accel'], 'Position',[30 110 150 140]);
        add_block('simulink/Sources/From Workspace', [modelName '/wheelYaw'], 'Position',[30 180 150 210]);

        % MATLAB Function block
        add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/IMU_Integrator'], ...
            'Position',[220 65 420 195]);

        % Sinks
        add_block('simulink/Sinks/To Workspace', [modelName '/p_out'], 'Position',[500 70 620 95]);
        add_block('simulink/Sinks/To Workspace', [modelName '/yaw_out'], 'Position',[500 150 620 175]);

        set_param([modelName '/gyro'], 'VariableName', 'gyro_m');
        set_param([modelName '/accel'], 'VariableName', 'accel_m');
        set_param([modelName '/wheelYaw'], 'VariableName', 'wheel_yaw');
        set_param([modelName '/p_out'], 'VariableName', 'p_out');
        set_param([modelName '/yaw_out'], 'VariableName', 'yaw_out');

        % Connect lines
        add_line(modelName, 'gyro/1', 'IMU_Integrator/1');
        add_line(modelName, 'accel/1', 'IMU_Integrator/2');
        add_line(modelName, 'wheelYaw/1', 'IMU_Integrator/3');
        add_line(modelName, 'IMU_Integrator/1', 'p_out/1');
        add_line(modelName, 'IMU_Integrator/2', 'yaw_out/1');

        % Configure sample time
        set_param(modelName, 'Solver', 'FixedStepDiscrete', 'FixedStep', num2str(Ts), 'StopTime', num2str(t(end)));

        % Insert code for MATLAB Function block
        mf = [modelName '/IMU_Integrator'];
        code = sprintf([ ...
            'function [p,yaw] = f(gyro,accel,wheelYaw)\\n' ...
            '%% IMU strapdown integrator (persistent state)\\n' ...
            '%% Inputs are row vectors [gx gy gz], [ax ay az], scalar wheelYaw\\n' ...
            'persistent q v pstate lastT\\n' ...
            'Ts = %.15g; g = %.15g; tau = %.15g;\\n' ...
            'if isempty(q)\\n' ...
            '  q = [1 0 0 0]; v = [0 0 0]; pstate = [0 0 0]; lastT = 0;\\n' ...
            'end\\n' ...
            '%% Propagate attitude\\n' ...
            'dq = q_from_delta_theta((gyro)*Ts);\\n' ...
            'q = q_mul(q, dq); q = q./norm(q);\\n' ...
            'yaw = yaw_from_q(q); q = q_from_yaw(yaw);\\n' ...
            '%% Acceleration to nav\\n' ...
            'R = q_to_R(q);\\n' ...
            'anav = (R*(accel'')'') + [0 0 -g];\\n' ...
            '%% Integrate\\n' ...
            'v = v + anav*Ts;\\n' ...
            'pstate = pstate + v*Ts + 0.5*anav*Ts^2;\\n' ...
            'pstate(3)=0; v(3)=0;\\n' ...
            '%% Complementary yaw fusion\\n' ...
            'alpha = exp(-Ts/max(tau,1e-6));\\n' ...
            'yaw = wrapToPi(alpha*yaw + (1-alpha)*wheelYaw);\\n' ...
            'q = q_from_yaw(yaw);\\n' ...
            'p = pstate;\\n' ...
            'end\\n' ...
            '\\n' ...
            'function dq = q_from_delta_theta(dtheta)\\n' ...
            'angle = norm(dtheta);\\n' ...
            'if angle &lt; 1e-12\\n' ...
            '  dq = [1, 0.5*dtheta];\\n' ...
            'else\\n' ...
            '  axis = dtheta/angle; half = 0.5*angle;\\n' ...
            '  dq = [cos(half), axis*sin(half)];\\n' ...
            'end\\n' ...
            'end\\n' ...
            '\\n' ...
            'function q = q_mul(a,b)\\n' ...
            'q = [a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4), ...\\n' ...
            '     a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3), ...\\n' ...
            '     a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2), ...\\n' ...
            '     a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1)];\\n' ...
            'end\\n' ...
            '\\n' ...
            'function R = q_to_R(q)\\n' ...
            'w=q(1); x=q(2); y=q(3); z=q(4);\\n' ...
            'R = [1-2*(y^2+z^2), 2*(x*y-w*z), 2*(x*z+w*y); ...\\n' ...
            '     2*(x*y+w*z), 1-2*(x^2+z^2), 2*(y*z-w*x); ...\\n' ...
            '     2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x^2+y^2)];\\n' ...
            'end\\n' ...
            '\\n' ...
            'function yaw = yaw_from_q(q)\\n' ...
            'w=q(1); x=q(2); y=q(3); z=q(4);\\n' ...
            'yaw = atan2(2*(w*z + x*y), 1-2*(y^2+z^2));\\n' ...
            'end\\n' ...
            '\\n' ...
            'function q = q_from_yaw(yaw)\\n' ...
            'half = 0.5*yaw; q = [cos(half), 0, 0, sin(half)];\\n' ...
            'end\\n'], Ts, g, params.tau_yaw);

        % Replace HTML entities inside MATLAB Function code with real chars for Simulink
        code = strrep(code, '&lt;', '<');
        set_param(mf, 'Script', code);

        save_system(modelName);
        fprintf('Built Simulink model: %s.slx\\n', modelName);
    catch ME
        warning("Simulink model creation failed: %s", ME.message);
    end
end

%% ===== Local functions =====
function [p_hist, v_hist, yaw_hist] = strapdown_integrate_matlab(t, gyro, accel, wheel_yaw, params)
N = size(t,1);
p = zeros(1,3); v = zeros(1,3); q = [1 0 0 0];
g_nav = [0 0 -params.g];

p_hist = zeros(N,3); v_hist = zeros(N,3); yaw_hist = zeros(N,1);

for k=1:N
    if k==1, dt = t(2)-t(1); else, dt = t(k)-t(k-1); end
    omega = gyro(k,:);
    dq = q_from_delta_theta(omega*dt);
    q = q_mul(q, dq); q = q./norm(q);
    yaw = yaw_from_q(q);
    if params.enforce_planar
        q = q_from_yaw(yaw);
    end

    R = q_to_R(q);
    a_nav = (R*accel(k,:)')' + g_nav;
    v = v + a_nav*dt;
    p = p + v*dt + 0.5*a_nav*dt^2;
    if params.enforce_planar
        p(3)=0; v(3)=0;
    end

    if params.fuse_yaw_with_wheel
        alpha = exp(-dt/max(params.tau_yaw,1e-6));
        yaw = wrapToPi(alpha*yaw + (1-alpha)*wheel_yaw(k));
        q = q_from_yaw(yaw);
    end

    p_hist(k,:) = p;
    v_hist(k,:) = v;
    yaw_hist(k) = yaw;
end
end

function dq = q_from_delta_theta(dtheta)
angle = norm(dtheta);
if angle < 1e-12
    dq = [1, 0.5*dtheta];
else
    axis = dtheta/angle;
    half = 0.5*angle;
    dq = [cos(half), axis*sin(half)];
end
end

function q = q_mul(a,b)
q = [a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4), ...
     a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3), ...
     a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2), ...
     a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1)];
end

function R = q_to_R(q)
w=q(1); x=q(2); y=q(3); z=q(4);
R = [1-2*(y^2+z^2), 2*(x*y-w*z), 2*(x*z+w*y); ...
     2*(x*y+w*z), 1-2*(x^2+z^2), 2*(y*z-w*x); ...
     2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x^2+y^2)];
end

function yaw = yaw_from_q(q)
w=q(1); x=q(2); y=q(3); z=q(4);
yaw = atan2(2*(w*z + x*y), 1-2*(y^2+z^2));
end

function q = q_from_yaw(yaw)
half = 0.5*yaw;
q = [cos(half), 0, 0, sin(half)];
end
