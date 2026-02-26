% Plant
s = tf('s');
G = 1 / (s * (s + 1));

% Proportional controller
Kp = 5;
L_prop = Kp * G;

figure;
nichols(L_prop);
grid on;
title('Nichols plot: proportional controller');

% First-order dynamic compensator
Kc    = 4;
tau_z = 0.5;
tau_p = 0.05;
Kdyn  = Kc * (tau_z * s + 1) / (tau_p * s + 1);
L_dyn = Kdyn * G;

figure;
nichols(L_dyn);
grid on;
title('Nichols plot: dynamic compensator');

% Closed-loop responses for comparison
T_prop = feedback(L_prop, 1);
T_dyn  = feedback(L_dyn, 1);

figure;
step(T_prop, T_dyn);
legend('Proportional', 'Dynamic compensator');
title('Step responses (for overshoot / settling comparison)');
