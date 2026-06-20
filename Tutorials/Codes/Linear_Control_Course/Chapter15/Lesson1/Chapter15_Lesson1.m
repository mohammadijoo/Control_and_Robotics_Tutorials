% Robot joint model parameters
J = 0.01;
B = 0.1;
K = 1.0;
Kp = 5.0;

s = tf('s');
G = K / (J * s^2 + B * s);
C = Kp;
L = C * G;

figure;
nyquist(L);
grid on;
title('Nyquist Plot of Robot Joint with P Control');

% Example: obtaining a linearized joint model from a robotics toolbox model
% (pseudo-code; requires Robotics System Toolbox and a defined robot model)
% robot = loadrobot('kinovaGen3');   % example robot
% q0 = homeConfiguration(robot);
% linSys = linearize(robotModelToSimulink(robot), q0); % conceptual step
% nyquist(linSys);
