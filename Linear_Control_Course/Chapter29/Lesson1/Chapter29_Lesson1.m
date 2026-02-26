% Motor parameters
R = 2.0;
L = 0.5;
J = 0.02;
b = 0.002;
Kt = 0.1;
Ke = 0.1;

Km = Kt / (b * R + Kt * Ke);
Tm = J * R / (b * R + Kt * Ke);

s = tf('s');

% Speed plant
G_omega = Km / (Tm * s + 1);

% Speed PI controller design
zeta = 0.8;
omega_n = 20;
Kp = (2 * zeta * omega_n * Tm - 1) / Km;
Ki = (Tm * omega_n^2) / Km;

C_speed = Kp + Ki / s;

T_speed = feedback(C_speed * G_omega, 1);

% Position loop: integrate speed
G_pos_eff = (1 / s) * T_speed;

omega_n_theta = 5;
tau_v = 1 / omega_n;
Kp_theta = tau_v * omega_n_theta^2;

C_pos = Kp_theta;

T_pos = feedback(C_pos * G_pos_eff, 1);

figure;
step(T_speed);
title('Closed-loop speed response');

figure;
step(T_pos);
title('Closed-loop position response');
grid on;
