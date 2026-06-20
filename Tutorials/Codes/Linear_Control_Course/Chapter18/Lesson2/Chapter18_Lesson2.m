% Robot joint parameters
J = 0.01;
b = 0.1;

% Plant: torque -> angle
P = tf(1, [J b 0]);

% PI controller
Kp = 5;
Ki = 10;
C = tf([Kp Ki], [1 0]);  % Kp + Ki/s

L = series(C, P);

% Disturbance transfer function (plant-input disturbance)
Gd = minreal(P / (1 + L));

w = logspace(-1, 2, 400);
[mag, phase, wout] = bode(Gd, w);
mag = squeeze(mag);

figure;
semilogx(wout, 20*log10(mag));
grid on;
xlabel('Frequency [rad/s]');
ylabel('Magnitude of Gd(jw) [dB]');
title('Disturbance rejection for plant-input disturbance');

% In a robotics workflow:
%  * Build a rigidBodyTree robot model.
%  * Use linearize() around an operating point to get P(s).
%  * Design C(s) and compute Gd(s) as above.
