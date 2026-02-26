s = tf('s');
G = 1/(s*(s + 1));

PM_des = 50;       % desired phase margin (deg)
wc_des = 4;        % desired crossover (rad/s)
safety = 5;        % extra safety margin (deg)

[mag, phase] = bode(G, wc_des);
mag = squeeze(mag);
phase = squeeze(phase);
PM0 = 180 + phase;               % uncompensated PM at wc_des
phi_req = PM_des - PM0 + safety; % required phase boost
phi_req = max(5, min(phi_req, 60));

phi_max = deg2rad(phi_req);
alpha = (1 - sin(phi_max))/(1 + sin(phi_max));
tau = 1/(wc_des*sqrt(alpha));

C0 = (tau*s + 1)/(alpha*tau*s + 1);   % lead without final gain
[magC0G, ~] = bode(C0*G, wc_des);
magC0G = squeeze(magC0G);
Kc = 1/magC0G;

C_lead = Kc*C0;
L = C_lead*G;

% Margins and Bode plots
margin(L);
grid on;

% Closed-loop step response
T = feedback(L, 1);
figure;
step(T);
title('Closed-loop step response with lead compensation');
