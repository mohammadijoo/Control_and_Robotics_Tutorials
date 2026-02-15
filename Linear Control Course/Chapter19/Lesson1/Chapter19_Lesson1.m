% Plant: G(s) = 1 / (s (s + 2))
s = tf('s');
G = 1 / (s * (s + 2));

% Lead compensator parameters
phi_req_deg = 30;
phi_safety_deg = 5;
phi_max_deg = phi_req_deg + phi_safety_deg;
phi_max = deg2rad(phi_max_deg);

alpha = (1 + sin(phi_max)) / (1 - sin(phi_max));

omega_c_star = 2;  % desired new crossover
T = 1 / (omega_c_star * sqrt(alpha));

% Lead compensator: C(s) = Kc (1 + alpha T s) / (1 + T s)
Kc = 1;
C_base = Kc * (1 + alpha * T * s) / (1 + T * s);

[mag, ~, ~] = bode(C_base * G, omega_c_star);
mag0 = squeeze(mag);
Kc = 1 / mag0;
C = Kc * C_base;

% Open-loop and closed-loop
L = C * G;
T_cl = feedback(L, 1);

figure; margin(L); grid on;
title('Bode and margins of compensated open loop');

figure; step(T_cl);
grid on;
title('Step response with lead compensation');

% Simulink hint:
% - Create a new model, place a Transfer Fcn block for G(s)
% - Place another Transfer Fcn block for C(s) with numerator [Kc * alpha*T, Kc]
%   and denominator [T, 1]
% - Connect blocks in series and close unity feedback loop
