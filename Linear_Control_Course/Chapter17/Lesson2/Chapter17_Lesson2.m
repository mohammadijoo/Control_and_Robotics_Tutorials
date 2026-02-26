% Parameters for L(s) = K / (s (T s + 1))
K = 10;
T = 0.1;
num = K;
den = [T 1 0];   % T s^2 + s

L = tf(num, den);

% Use margin to get gain and phase crossover frequencies
[GM, PM, w_pc, w_gc] = margin(L);

fprintf('Gain crossover w_gc = %.3f rad/s\n', w_gc);
fprintf('Phase crossover w_pc = %.3f rad/s\n', w_pc);
fprintf('Gain margin = %.3f (%.2f dB)\n', GM, 20*log10(GM));
fprintf('Phase margin = %.2f deg\n', PM);

% Bode plot with margins
figure;
margin(L);
grid on;
title('Open-loop Bode plot with gain/phase margins');

% Simulink hint:
% In a Simulink model of a robotic joint (plant + controller),
% 1) Linearize the model around an operating point using "linearize".
% 2) Use "bode" or "margin" on the resulting LTI system.
