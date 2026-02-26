% Chapter4_Lesson1.m
% Autonomous Mobile Robots - Chapter 4 (Mobile Robot Dynamics Applied)
% Lesson 1: When Dynamics Matter for AMR
%
% This script compares:
%   (1) a kinematic unicycle model driven by (v_cmd, w_cmd)
%   (2) a simple dynamic model where (v,w) evolve due to bounded wheel torques
% It also optionally creates a small Simulink model programmatically.

clear; clc;

dt = 0.002;
T  = 10.0;
ts = 0:dt:T;

% Parameters
p.m = 30.0;
p.Iz = 1.2;
p.rw = 0.10;
p.b  = 0.50;
p.bv = 6.0;
p.bw = 0.25;
p.tau_max = 3.0;
p.kv = 8.0;
p.kw = 1.2;

% States
xk = [0;0;0];        % [px; py; th]
xd = [0;0;0;0;0];    % [px; py; th; v; w]

Xk = zeros(3, numel(ts));
Xd = zeros(5, numel(ts));
U  = zeros(2, numel(ts));

for i = 1:numel(ts)
    t = ts(i);
    u = cmd_profile(t);
    U(:,i) = u;

    Xk(:,i) = xk;
    Xd(:,i) = xd;

    xk = rk4_step(@(tt,xx) kin_rhs(tt, xx, u), t, xk, dt);
    xk(3) = wrap_pi(xk(3));

    xd = rk4_step(@(tt,xx) dyn_rhs(tt, xx, u, p), t, xd, dt);
    xd(3) = wrap_pi(xd(3));
end

figure; plot(Xk(1,:), Xk(2,:), 'LineWidth', 1.2); hold on;
plot(Xd(1,:), Xd(2,:), 'LineWidth', 1.2);
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Trajectory: kinematic vs dynamic');
legend('Kinematic (ideal v,w)', 'Dynamic (torque-limited)');

figure; plot(ts, U(1,:), 'LineWidth', 1.2); hold on;
plot(ts, Xd(4,:), 'LineWidth', 1.2);
grid on; xlabel('t [s]'); ylabel('v [m/s]');
title('Linear speed tracking'); legend('v\_cmd', 'v');

figure; plot(ts, U(2,:), 'LineWidth', 1.2); hold on;
plot(ts, Xd(5,:), 'LineWidth', 1.2);
grid on; xlabel('t [s]'); ylabel('w [rad/s]');
title('Yaw-rate tracking'); legend('w\_cmd', 'w');

% ----------------------------
% Optional: create a Simulink model programmatically
% (This produces a simple unicycle-with-first-order actuator block diagram.)
% ----------------------------
make_simulink = false;
if make_simulink
    mdl = 'Chapter4_Lesson1_SimpleActuatorModel';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);

    add_block('simulink/Sources/Step', [mdl '/v_cmd']);
    set_param([mdl '/v_cmd'], 'Time', '0', 'Before', '0.2', 'After', '1.1');

    add_block('simulink/Sources/Step', [mdl '/w_cmd']);
    set_param([mdl '/w_cmd'], 'Time', '5', 'Before', '0.6', 'After', '1.5');

    add_block('simulink/Continuous/Transfer Fcn', [mdl '/Act_v']);
    set_param([mdl '/Act_v'], 'Numerator', '1', 'Denominator', '[0.2 1]');

    add_block('simulink/Continuous/Transfer Fcn', [mdl '/Act_w']);
    set_param([mdl '/Act_w'], 'Numerator', '1', 'Denominator', '[0.3 1]');

    add_block('simulink/Math Operations/Trigonometric Function', [mdl '/cos']);
    set_param([mdl '/cos'], 'Operator', 'cos');
    add_block('simulink/Math Operations/Trigonometric Function', [mdl '/sin']);
    set_param([mdl '/sin'], 'Operator', 'sin');

    add_block('simulink/Math Operations/Product', [mdl '/v*cos']);
    add_block('simulink/Math Operations/Product', [mdl '/v*sin']);

    add_block('simulink/Continuous/Integrator', [mdl '/Int_x']);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_y']);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_th']);

    add_block('simulink/Sinks/Scope', [mdl '/Scope']);

    % Wiring
    add_line(mdl, 'v_cmd/1', 'Act_v/1');
    add_line(mdl, 'w_cmd/1', 'Act_w/1');

    add_line(mdl, 'Act_v/1', 'v*cos/1');
    add_line(mdl, 'Act_v/1', 'v*sin/1');

    add_line(mdl, 'Int_th/1', 'cos/1');
    add_line(mdl, 'Int_th/1', 'sin/1');

    add_line(mdl, 'cos/1', 'v*cos/2');
    add_line(mdl, 'sin/1', 'v*sin/2');

    add_line(mdl, 'v*cos/1', 'Int_x/1');
    add_line(mdl, 'v*sin/1', 'Int_y/1');

    add_line(mdl, 'Act_w/1', 'Int_th/1');

    add_line(mdl, 'Int_x/1', 'Scope/1');
    add_line(mdl, 'Int_y/1', 'Scope/2');
    add_line(mdl, 'Int_th/1', 'Scope/3');

    set_param(mdl, 'StopTime', num2str(T));
    save_system(mdl);
end

% ---------- Local functions ----------
function u = cmd_profile(t)
    if t <= 5.0
        v_cmd = 0.2 + 0.18*t;
        w_cmd = 0.6;
    else
        v_cmd = 1.1;
        w_cmd = 1.5;
    end
    u = [v_cmd; w_cmd];
end

function dx = kin_rhs(~, x, u)
    px = x(1); py = x(2); th = x(3);
    v = u(1); w = u(2);
    dx = [v*cos(th); v*sin(th); w];
end

function dx = dyn_rhs(~, x, u, p)
    px = x(1); py = x(2); th = x(3);
    v  = x(4); w  = x(5);

    v_ref = u(1); w_ref = u(2);

    tau_sum  = p.kv*(v_ref - v);
    tau_diff = p.kw*(w_ref - w);

    tau_R = 0.5*(tau_sum + tau_diff);
    tau_L = 0.5*(tau_sum - tau_diff);

    tau_R = sat(tau_R, -p.tau_max, p.tau_max);
    tau_L = sat(tau_L, -p.tau_max, p.tau_max);

    Fx = (tau_R + tau_L)/p.rw;
    Mz = (p.b/(2*p.rw))*(tau_R - tau_L);

    v_dot = (Fx - p.bv*v)/p.m;
    w_dot = (Mz - p.bw*w)/p.Iz;

    dx = [v*cos(th); v*sin(th); w; v_dot; w_dot];
end

function xn = rk4_step(f, t, x, dt)
    k1 = f(t, x);
    k2 = f(t + 0.5*dt, x + 0.5*dt*k1);
    k3 = f(t + 0.5*dt, x + 0.5*dt*k2);
    k4 = f(t + dt, x + dt*k3);
    xn = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function a = wrap_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end

function y = sat(x, lo, hi)
    y = min(max(x, lo), hi);
end
