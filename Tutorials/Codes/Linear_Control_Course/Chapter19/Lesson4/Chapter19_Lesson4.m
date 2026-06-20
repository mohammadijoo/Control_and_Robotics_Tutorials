% Plant: second-order joint model G(s) = Km / (J s^2 + B s)
J = 0.01;
B = 0.1;
Km = 0.5;

s = tf('s');
G = Km / (J * s^2 + B * s);

% Lead design (parameters from Bode-based procedure)
wc_des = 10;      % rad/s
alpha = 0.2;      % z1/p1 = alpha < 1

z1 = wc_des / sqrt(alpha);
p1 = wc_des * sqrt(alpha);

G_lead = (s + z1) / (s + p1);

% Choose gain Kc so that |L(j wc_des)| ≈ 1
[magG, ~] = bode(G, wc_des);
[magLead, ~] = bode(G_lead, wc_des);
magG = squeeze(magG);
magLead = squeeze(magLead);
Kc = 1 / (magG * magLead);

% Lag design for improved low-frequency error
beta = 10;
wz2 = wc_des / 10;
wp2 = wz2 / beta;
G_lag = (s + wz2) / (s + wp2);

Gc = Kc * G_lead * G_lag;
L = Gc * G;
T = feedback(L, 1);

figure;
margin(L);
title('Bode and margins of lead-lag compensated open-loop');

figure;
step(T);
grid on;
title('Closed-loop step response with lead-lag compensator');
xlabel('Time [s]');
ylabel('Joint position [rad]');

% In Simulink:
% - Implement G(s) as a Transfer Fcn block.
% - Implement G_lead(s) and G_lag(s) as cascaded Transfer Fcn blocks.
% - Use a Step block for reference and a Sum block for the error.
% - Optionally interface with Robotics System Toolbox rigidBodyTree models.
