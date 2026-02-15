J = 0.01;
b = 0.1;
Km = 1.0;

zeta_f = 0.05;
w_f = 200;       % rad/s

s = tf('s');

P0 = Km / (J * s^2 + b * s);
Gf = (w_f^2) / (s^2 + 2 * zeta_f * w_f * s + w_f^2);
Ptrue = P0 * Gf;

Delta_m = Gf - 1;   % multiplicative deviation: Ptrue/P0 - 1

Kp = 5;
C = Kp;

L0 = C * P0;
T0 = feedback(L0, 1);

alpha = 1 / (2 * zeta_f);
Wm = alpha * (s / w_f) / (1 + s / w_f);

w = logspace(0, 4, 500);

[magDm, ~, ~] = bode(Delta_m, w);
[magWm, ~, ~] = bode(Wm, w);
[magT0, ~, ~] = bode(T0, w);

magDm = squeeze(magDm);
magWm = squeeze(magWm);
magT0 = squeeze(magT0);

figure;
loglog(w, magDm, 'LineWidth', 1.5); hold on;
loglog(w, magWm, '--', 'LineWidth', 1.5);
xlabel('Frequency (rad/s)');
ylabel('Magnitude');
legend('|Delta_m(jw)|', '|W_m(jw)|');
grid on;

figure;
loglog(w, magWm .* magT0, 'LineWidth', 1.5);
xlabel('Frequency (rad/s)');
ylabel('|W_m(jw) T_0(jw)|');
grid on;

% In a robotics workflow, J and b can be derived from a rigid-body tree model:
%   robot = importrobot('yourRobot.urdf');
%   J = someFunctionOf(robot, jointIndex);
% and then the same uncertainty analysis can be repeated for each joint.
