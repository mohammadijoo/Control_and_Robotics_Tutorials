% Chapter2_Lesson3.m
% Autonomous Mobile Robots — Chapter 2, Lesson 3
% Car-Like / Ackermann Steering Kinematics
%
% This script implements:
% 1) Bicycle kinematic model.
% 2) Exact integration for constant (v, delta) over dt.
% 3) Ackermann wheel angles from a virtual steering delta.
%
% MATLAB toolboxes often used for AMR:
% - Robotics System Toolbox (ROS, SE(2)/SE(3), planners)
% - Navigation Toolbox (maps, localization, path following)
% - Simulink (system integration)
%
% Run:
%   Chapter2_Lesson3

clear; clc;

L = 2.7;   % wheelbase [m]
W = 1.6;   % track width [m]

p = [0; 0; 0]; % [x; y; theta]

% controls: [v, delta, dt]
U = [
    2.0, deg2rad(15.0), 1.0;
    2.0, deg2rad(15.0), 1.0;
    2.0, deg2rad(0.0),  1.0;
    2.0, deg2rad(0.0),  1.0
];

P = zeros(size(U,1)+1, 3);
P(1,:) = p';

for k = 1:size(U,1)
    v = U(k,1); delta = U(k,2); dt = U(k,3);
    p = step_bicycle_exact(p, v, delta, L, dt);
    P(k+1,:) = p';
end

disp("Final pose (exact):");
disp(P(end,:));

delta_virtual = deg2rad(15.0);
[dl, dr] = ackermann_wheel_angles_from_virtual(delta_virtual, L, W);
fprintf("Virtual delta = %.3f deg\n", rad2deg(delta_virtual));
fprintf("Ackermann delta_left  = %.3f deg\n", rad2deg(dl));
fprintf("Ackermann delta_right = %.3f deg\n", rad2deg(dr));

% Optional plot
figure; plot(P(:,1), P(:,2), '-o'); axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Bicycle model simulation (exact integration)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Local functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function a = wrap_to_pi(a)
    a = mod(a + pi, 2*pi);
    if a < 0, a = a + 2*pi; end
    a = a - pi;
end

function kappa = curvature_from_steering(delta, L)
    kappa = tan(delta) / L;
end

function [dl, dr] = ackermann_wheel_angles_from_virtual(delta, L, W)
    kappa = curvature_from_steering(delta, L);
    if abs(kappa) < 1e-12
        dl = 0.0; dr = 0.0; return;
    end
    R = 1.0 / kappa; % signed
    dl = atan2(L, (R - W/2));
    dr = atan2(L, (R + W/2));
end

function p2 = step_bicycle_exact(p, v, delta, L, dt)
    omega = v * tan(delta) / L;
    if abs(omega) < 1e-10
        x2 = p(1) + dt * v * cos(p(3));
        y2 = p(2) + dt * v * sin(p(3));
        t2 = wrap_to_pi(p(3) + dt * omega);
        p2 = [x2; y2; t2];
        return;
    end
    th2 = p(3) + omega * dt;
    x2 = p(1) + (v/omega) * (sin(th2) - sin(p(3)));
    y2 = p(2) + (v/omega) * (-cos(th2) + cos(p(3)));
    t2 = wrap_to_pi(th2);
    p2 = [x2; y2; t2];
end
