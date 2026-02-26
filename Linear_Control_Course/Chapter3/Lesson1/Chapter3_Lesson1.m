% Mass-spring-damper LTI model
m = 1.0;
b = 0.4;
k = 2.0;

A = [0 1;
    -k/m -b/m];
B = [0;
     1/m];
C = [1 0];
D = 0;

sys = ss(A, B, C, D);   % state-space LTI system

% Step response (force step)
figure;
step(sys);
title('Mass-spring-damper step response');

% Robotics context (conceptual):
% robot = importrobot('exampleRobot.urdf');
% linSys = linearize(robot, operatingPoint); % approximate LTI model around a pose
