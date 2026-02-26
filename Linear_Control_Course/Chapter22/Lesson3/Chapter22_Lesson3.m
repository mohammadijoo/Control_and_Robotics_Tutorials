% Robot joint parameters
J = 0.01;
B = 0.1;

s = tf('s');
P = 1 / (J * s^2 + B * s);

Kp = 50;
Kd = 2;
C = Kp + Kd * s;

L = C * P;
S = 1 / (1 + L);
T = L / (1 + L);

% Bode plot of S and T
w = logspace(-1, 3, 400);
figure;
bodemag(S, T, w);
legend('|S(jw)|', '|T(jw)|');
grid on;
title('Sensitivity and Complementary Sensitivity');

% Using loopsens for full closed-loop maps
% (useful when adding disturbance and noise inputs)
CL = loopsens(P, C);
% CL.Si is S(s), CL.So is output sensitivity, CL.Ti/To are complementary sensitivities
