% Proportional control of G(s) = 1/s
K = 10;
s = tf('s');
G = 1/s;
C = K;
L = C*G;

% Classical margins
[gm, pm, wg, wp] = margin(L);

% Delay margin from phase margin and gain crossover
Td = deg2rad(pm)/wg;

fprintf('Phase margin = %.2f deg\n', pm);
fprintf('Gain crossover = %.2f rad/s\n', wg);
fprintf('Approximate delay margin = %.4f s\n', Td);

% Verify with Pade approximation of the delay
[num_pade, den_pade] = pade(Td, 1);
DelayApprox = tf(num_pade, den_pade);
L_delayed = L * DelayApprox;
T_cl = feedback(L_delayed, 1);
pole(T_cl)
