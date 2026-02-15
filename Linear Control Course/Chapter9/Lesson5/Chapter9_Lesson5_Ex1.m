% Import robot model and linearize a joint about a configuration
robot = importrobot('my_robot.urdf');
robot.DataFormat = 'row';
q0 = homeConfiguration(robot);

% Create a simple joint-space PD servo model in Simulink or as a
% linearized plant using dynamics functions (schematic example):
% [A,B,C,D] = linearizeJoint(robot, jointIndex, q0);
% G_joint = tf(ss(A,B,C,D));

% Once G_joint is available:
figure;
rlocus(G_joint);
title('Root locus of robotic joint servo loop');
grid on;
