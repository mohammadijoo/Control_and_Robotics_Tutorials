s = tf('s');

% Double integrator plant
G = 1 / s^2;

% Design choices (same as Python example)
zeta = 0.6;
ts_des = 1.0;
omega_n = 4 / (zeta * ts_des);

Kd = 2 * zeta * omega_n;
Kp = omega_n^2;

C_pd = Kd * s + Kp;

% Closed-loop tracking without prefilter
T0 = feedback(C_pd * G, 1);

% First-order prefilter F(s)
omega_f = 0.7 * omega_n;
F = omega_f / (s + omega_f);

% Prefiltered tracking transfer function
T = minreal(F * T0);

figure;
step(T0, 'b--', T, 'r-', 3); % simulate 3 seconds
grid on;
legend('Without prefilter', 'With prefilter');
title('Set-point tracking with PD controller and prefilter');

% In Simulink:
%  - Use Transfer Fcn blocks for G(s), C_pd(s), and F(s)
%  - Connect F(s) in series with the reference input
%  - Use Sum blocks to form the error and feedback paths
