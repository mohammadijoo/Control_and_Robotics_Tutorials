% Physical parameters
M = 1.0;
B = 0.5;
K = 4.0;

% PD gains
Kp = 16.0;
Kd = 4.0;

s = tf('s');

% Plant: 1 / (M s^2 + B s + K)
G = 1 / (M * s^2 + B * s + K);

% PD controller: C(s) = Kp + Kd s
C = Kp + Kd * s;

% Closed-loop transfer function
T = feedback(C * G, 1);

% Step response
figure;
step(T);
grid on;
title('PD-Controlled Mass-Spring-Damper Step Response');

% Compute effective natural frequency and damping ratio
omega_n = sqrt((K + Kp) / M);
zeta = (B + Kd) / (2 * sqrt(M * (K + Kp)));

disp(['Effective omega_n = ', num2str(omega_n)]);
disp(['Effective zeta    = ', num2str(zeta)]);

% In Simulink, a similar PD law is often used for robot joint control,
% possibly together with the Robotics System Toolbox to model manipulators.
