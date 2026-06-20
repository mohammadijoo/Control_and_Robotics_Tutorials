% Thermal system parameters
tau_th = 300;    % s
K_th   = 5;      % K per unit input

s = tf('s');
G_th = K_th / (tau_th * s + 1);

% Desired specs
zeta   = 0.7;
Ts_des = 1200;   % s
omega_n = 4 / (zeta * Ts_des);

% PI gains from matching
Kp = (2 * zeta * omega_n * tau_th - 1) / K_th;
Ki = (omega_n^2 * tau_th) / K_th;

% PI controller C(s) = Kp + Ki/s
C = Kp + Ki / s;

% Closed-loop
T_cl = feedback(C * G_th, 1);

figure;
step(T_cl, 0:10:4000);
grid on;
title('Closed-loop step response of thermal system with PI control');
ylabel('Temperature change [K]');
xlabel('Time [s]');
