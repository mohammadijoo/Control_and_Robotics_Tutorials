% Chapter1_Lesson2.m
% Autonomous Mobile Robots — Chapter 1, Lesson 2
% State variables for mobile robots (pose, velocity, uncertainty)

clear; clc;

v = 0.8;
omega = 0.35;
dt = 0.05;
N = 200;

sigma_v = 0.05;
sigma_w = 0.03;
Q_u = diag([sigma_v^2, sigma_w^2]);

p = [0; 0; 0];
P = diag([0.02^2, 0.02^2, (2*pi/180)^2]);

for k = 1:N
    p = integrate_unicycle_midpoint(p, v, omega, dt);
    [F, G] = jacobians_euler(p, v, dt);
    P = F*P*F' + G*Q_u*G';
end

disp('Final pose [x; y; theta] =');
disp(p);
disp('Covariance diagonal =');
disp(diag(P).');

function tf = dt_is_nonpositive(dt)
    tf = (dt == 0) || (dt ~= abs(dt));
end

function theta = wrap_angle(theta)
    theta = mod(theta + pi, 2*pi) - pi;
end

function p_next = integrate_unicycle_midpoint(p, v, omega, dt)
    if dt_is_nonpositive(dt)
        error('dt must be positive');
    end
    th = p(3);
    th_mid = th + 0.5*dt*omega;
    x_next = p(1) + dt*v*cos(th_mid);
    y_next = p(2) + dt*v*sin(th_mid);
    th_next = wrap_angle(th + dt*omega);
    p_next = [x_next; y_next; th_next];
end

function [F, G] = jacobians_euler(p, v, dt)
    th = p(3);
    c = cos(th);
    s = sin(th);

    F = eye(3);
    F(1,3) = -dt*v*s;
    F(2,3) =  dt*v*c;

    G = zeros(3,2);
    G(1,1) = dt*c;
    G(2,1) = dt*s;
    G(3,2) = dt;
end
