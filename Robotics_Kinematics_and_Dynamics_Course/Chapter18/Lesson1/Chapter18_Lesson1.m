% Parameters
l1 = 1.0;
l2 = 0.8;

% Time grid
T = 2.0;
N = 200;
t = linspace(0, T, N+1);

% Joint trajectory parameters
q10 = 0.0; q20 = 0.5;
a1 = 0.8; a2 = -0.3;
b1 = -0.2; b2 = 0.15;

% Joint trajectories
q1 = q10 + a1 * t + b1 * t.^2;
q2 = q20 + a2 * t + b2 * t.^2;

% Forward kinematics for planar 2R
x = l1 * cos(q1) + l2 * cos(q1 + q2);
y = l1 * sin(q1) + l2 * sin(q1 + q2);

% Plot in task space
figure;
plot(x, y, '-');
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('End-effector trajectory');
      
