% Define transfer function
s = tf('s');
G = 100 * (s + 10) / (s * (s + 1) * (s + 100));

% Exact Bode plot
w = logspace(-2, 3, 200);
[mag, phase] = bode(G, w);
mag = squeeze(mag);
phase = squeeze(phase);

% Approximate magnitude using slope rules
mag_db_approx = 20*log10(100) - 20*log10(w);  % integrator + gain

% Adjust slopes at corners:
% Pole at 1 rad/s
idx = w >= 1;
mag_db_approx(idx) = mag_db_approx(idx) - 20*log10(w(idx) / 1);

% Zero at 10 rad/s
idx = w >= 10;
mag_db_approx(idx) = mag_db_approx(idx) + 20*log10(w(idx) / 10);

% Pole at 100 rad/s
idx = w >= 100;
mag_db_approx(idx) = mag_db_approx(idx) - 20*log10(w(idx) / 100);

figure;
semilogx(w, 20*log10(mag), 'LineWidth', 1.5); hold on;
semilogx(w, mag_db_approx, '--', 'LineWidth', 1.5);
grid on;
xlabel('omega [rad/s]');
ylabel('Magnitude [dB]');
legend('Exact', 'Approximate');

% Phase (exact from bode)
figure;
semilogx(w, phase, 'LineWidth', 1.5);
grid on;
xlabel('omega [rad/s]');
ylabel('Phase [deg]');
title('Phase of G(s)');
