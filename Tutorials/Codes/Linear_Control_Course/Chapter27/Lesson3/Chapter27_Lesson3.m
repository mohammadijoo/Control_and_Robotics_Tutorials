% Physical parameters
J = 0.01;
b = 0.1;

% PI controller gains
Kp = 20;
Ki = 50;

s = tf('s');
G = 1 / (J * s^2 + b * s);
C = Kp + Ki / s;

L = C * G;
S = 1 / (1 + L);
T = L / (1 + L);

% Disturbance at plant input: Gd(s) = S(s) G(s)
Gd_in = minreal(S * G);

% Bode magnitude of S and Gd_in
w = logspace(-1, 2, 500);
[magS, ~] = bode(S, w);
[magGd, ~] = bode(Gd_in, w);

magS_db = 20 * log10(squeeze(magS));
magGd_db = 20 * log10(squeeze(magGd));

figure;
semilogx(w, magS_db, 'b', w, magGd_db, 'r');
grid on;
xlabel('Frequency w [rad/s]');
ylabel('Magnitude [dB]');
legend('|S(jw)|', '|Gd\_in(jw)|', 'Location', 'SouthWest');
title('Sensitivity and Disturbance Transfer');

% Time-domain simulation of step load disturbance
t = linspace(0, 5, 1000);
D0 = 1.0;
d_step = D0 * ones(size(t));

[theta, t_out] = lsim(Gd_in, d_step, t);
theta_ss = theta(end);

fprintf('Approx. steady-state position error [rad]: %.6f\n', theta_ss);
