% Plant and controller (continuous time)
s  = tf('s');
P1 = 1 / (s * (s + 1));

Kc = 12;
zi = 0.8;
zl = 6;
pl = 1.5;

Ci = (s + zi) / s;
Cl = (s + zl) / (s + pl);
C1 = Kc * Ci * Cl;

L1 = C1 * P1;
T1 = feedback(L1, 1);

figure;
margin(L1); grid on;
title('Loop-shaped open-loop for position servo');

figure;
step(T1);
grid on;
title('Closed-loop step response for position servo');

% In Simulink, place blocks for P1, Ci, Cl and connect them
% in feedback; use "To Workspace" blocks to compare with this script.
