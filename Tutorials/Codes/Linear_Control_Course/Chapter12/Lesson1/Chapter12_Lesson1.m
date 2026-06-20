% Specifications
Mp_star = 0.10;
Ts_star = 2.0;
alpha   = 4.0;

% Step 1: zeta from Mp*
lnMp = log(Mp_star);
zeta = -lnMp / sqrt(pi^2 + lnMp^2);

% Step 2: omega_n from Ts*
omega_n = 4 / (zeta * Ts_star);

% Step 3: choose p3 and compute gains
p3 = alpha * zeta * omega_n;
Kd = 2*zeta*omega_n + p3 - 1;
Kp = omega_n^2 + 2*zeta*omega_n*p3;
Ki = omega_n^2 * p3;

fprintf('Kp = %.3f, Ki = %.3f, Kd = %.3f\n', Kp, Ki, Kd);

% Plant G(s) = 1/(s*(s+1))
s = tf('s');
G = 1/(s*(s+1));

% PID controller
C = Kp + Ki/s + Kd*s;

% Closed-loop transfer function
T = feedback(C*G, 1);

% Step response and time-domain characteristics
figure;
step(T);
grid on;
title('Closed-loop Step Response with Designed PID');

info = stepinfo(T);
disp(info);
