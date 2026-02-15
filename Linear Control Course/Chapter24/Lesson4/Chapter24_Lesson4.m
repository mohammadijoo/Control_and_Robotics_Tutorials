% Joint-like plant and PD controller
s = tf('s');
P = 1 / (s * (s + 1));
Kp = 40;
Kd = 2;
C = Kp + Kd * s;

L0 = C * P;

% Classical stability margins
[gm, pm_deg, wgc, wpc] = margin(L0);
pm_rad = pm_deg * pi / 180;
D_max = pm_rad / wgc;

fprintf('Phase margin = %.2f deg\n', pm_deg);
fprintf('Gain crossover wgc = %.2f rad/s\n', wgc);
fprintf('Approximate delay margin D_max = %.4f s\n', D_max);

% Use first-order Padé approximation for a nominal delay
L_nom = 0.03;  % 30 ms
[num_d, den_d] = pade(L_nom, 1);
D_pade = tf(num_d, den_d);
L = C * P * D_pade;
T = feedback(L, 1);

figure;
step(T);
title('Step response with 30 ms nominal delay (Padé approx)');
grid on;

% Simulink: implement P, C, and a Transport Delay block:
%  - Use "Transfer Fcn" blocks for P and C
%  - Insert a "Transport Delay" block with DelayTime = L_nom
%  - Close unity feedback and simulate response
