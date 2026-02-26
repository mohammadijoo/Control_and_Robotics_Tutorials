% Example plant for a robotic joint axis
s = tf('s');
G = 1 / (s * (s + 1) * (s + 2));

K = 2;                 % nominal gain
L = K * G;

figure;
nyquist(L);
hold on;
plot(-1, 0, 'rx', 'MarkerSize', 8, 'LineWidth', 2);
grid on;
axis equal;
title(sprintf('Nyquist plot for L(s), K = %.2f', K));

% Compute gain and phase margins (Nyquist/Bode-consistent)
[GM, PM, Wcg, Wcp] = margin(L);
fprintf('Gain margin GM = %.2f (at w = %.3f rad/s)\n', GM, Wcg);
fprintf('Phase margin PM = %.2f deg (at w = %.3f rad/s)\n', PM, Wcp);

% Approximate d_min on a frequency grid
omega = logspace(-2, 2, 1000);
Lw = squeeze(freqresp(L, omega));
z = 1 + Lw;
d = abs(z);
[d_min, idx] = min(d);
fprintf('Approx d_min = %.4f at omega = %.3f rad/s\n', d_min, omega(idx));

% Simulink hint:
% - In Simulink, linearize a robot model using linmod or linearize (Robotics System Toolbox),
%   then use bode, nyquist, and margin on the linearized open-loop model.
