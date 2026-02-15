% MATLAB script for Nichols-based sensitivity analysis

s = tf('s');

J = 0.01;
B = 0.1;
G = 1 / (J * s^2 + B * s);     % Joint dynamics

Kp = 20;
Kd = 1;
C = Kd * s + Kp;               % PD controller

L = C * G;
S = feedback(1, L);            % S(s) = 1 / (1 + L)
T = feedback(L, 1);            % T(s) = L / (1 + L)

w = logspace(-1, 3, 400);
[magS, phaseS] = bode(S, w);
magS = squeeze(magS);
Ms = max(magS);
Ms_dB = 20 * log10(Ms);

fprintf('Peak sensitivity Ms = %.3f (%.2f dB)\n', Ms, Ms_dB);

figure;
nichols(L, w);
grid on;
title('Nichols Plot of L(s) for Robotic Joint Servo');

% In Robotics System Toolbox, a more realistic G(s) can be obtained by:
% robot = importrobot('someRobot.urdf');
% linSys = linearize(robotSimModel, operatingPoint);
% and then L = C * linSys for Nichols analysis.
