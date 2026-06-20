% Plant
K = 1.0;
tau = 0.5;
s = tf('s');
G_p = K / (tau * s + 1);

% Desired dynamics
zeta = 0.7;
omega_n = 3.0;

% PI gains
Kp = (2 * zeta * omega_n * tau - 1) / K;
Ki = (tau * omega_n^2) / K;

fprintf('Kp = %.3f, Ki = %.3f\n', Kp, Ki);

G_c = Kp + Ki / s;      % PI controller
T   = feedback(G_c * G_p, 1);  % closed loop

figure;
step(T);
grid on;
title('PI-controlled first-order plant');
