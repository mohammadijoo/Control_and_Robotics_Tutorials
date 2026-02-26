% Chapter 3 — Nonholonomic Motion and Feasibility for AMR
% Lesson 1 — Nonholonomic Constraints in Wheeled Robots (applied view)
%
% This script simulates differential-drive kinematics and checks the
% lateral no-slip Pfaffian constraint residual.
%
% Tooling notes (MATLAB/Simulink):
%  - Robotics System Toolbox provides differentialDriveKinematics,
%    controllerPurePursuit, navPath, etc.
%  - In Simulink, a comparable model can be built with Integrator blocks
%    and a MATLAB Function block implementing the unicycle dynamics.

clear; clc; close all;

T  = 16.0;
dt = 0.01;
r  = 0.10;   % wheel radius [m]
b  = 0.22;   % half track width [m]

N = floor(T/dt) + 1;
t = linspace(0, T, N);

x  = zeros(1,N);
y  = zeros(1,N);
th = zeros(1,N);
v  = zeros(1,N);
w  = zeros(1,N);
res = zeros(1,N);

for k = 1:N-1
    [omL, omR] = wheel_profile(t(k));
    [v(k), w(k)] = diff_drive_twist(omL, omR, r, b);

    x_dot = v(k)*cos(th(k));
    y_dot = v(k)*sin(th(k));
    res(k) = -sin(th(k))*x_dot + cos(th(k))*y_dot;

    % Euler integration
    x(k+1)  = x(k)  + dt*x_dot;
    y(k+1)  = y(k)  + dt*y_dot;
    th(k+1) = wrapToPi(th(k) + dt*w(k));
end
v(end) = v(end-1);
w(end) = w(end-1);
x_dot = v(end)*cos(th(end));
y_dot = v(end)*sin(th(end));
res(end) = -sin(th(end))*x_dot + cos(th(end))*y_dot;

fprintf('max |constraint residual| = %.3e\n', max(abs(res)));
fprintf('final pose: x=%.3f y=%.3f theta=%.3f rad\n', x(end), y(end), th(end));

figure;
plot(x, y, 'LineWidth', 1.5);
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Differential-drive trajectory (ideal no-slip)');

figure;
plot(t, res, 'LineWidth', 1.2);
grid on;
xlabel('t [s]'); ylabel('constraint residual');
title('Lateral no-slip Pfaffian residual (should be ~0)');

% ---------------- Local functions ----------------

function [v, w] = diff_drive_twist(omL, omR, r, b)
    v = 0.5*r*(omR + omL);
    w = 0.5*r*(omR - omL)/b;
end

function [omL, omR] = wheel_profile(t)
    if t < 4.0
        omL = 6.0; omR = 6.0;
    elseif t < 8.0
        omL = 3.0; omR = 7.0;
    elseif t < 12.0
        omL = 7.0; omR = 3.0;
    else
        omL = 5.0; omR = 5.0;
    end
end

function a = wrapToPi(a)
    a = mod(a + pi, 2*pi);
    if a < 0
        a = a + 2*pi;
    end
    a = a - pi;
end
