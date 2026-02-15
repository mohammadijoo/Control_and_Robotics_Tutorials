tau = 0.5;
G = tf(1, [tau 1]);     % G(s) = 1 / (tau s + 1)

Kp = 4;
Ki = 5;
C  = tf([Kp Ki], [1 0]); % PI controller

L = C * G;
S = feedback(1, L);     % S = 1 / (1 + L)
T = feedback(L, 1);     % T = L / (1 + L)

t = 0:0.01:5;

% 1) Tracking: y for step in r
[y_track, t1] = step(T, t);

% 2) Regulation: y for step in output disturbance
[y_reg, t2] = step(S, t);

figure;
plot(t1, y_track, 'LineWidth', 1.5); hold on;
plot(t2, y_reg, 'LineWidth', 1.5);
yline(1, '--', 'Reference r(t) = 1');
yline(0, ':', 'Regulation target y(t) = 0');
xlabel('Time [s]'); ylabel('Output y(t)');
legend('Tracking (step r)', 'Regulation (step d)', ...
       'Location', 'Best');
grid on;
title('Tracking vs Regulation in MATLAB');
