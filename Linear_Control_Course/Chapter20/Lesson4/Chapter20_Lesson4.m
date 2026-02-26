% Plant and feedback controller
s = tf('s');
G = 1 / (s * (s + 1));

Kp = 30; Ki = 70; Kd = 2;
C = Kp + Ki / s + Kd * s;

L = C * G;

% Disturbance transfer function (independent of Q)
Tyd = minreal(G / (1 + L));

% Desired second-order tracking model
zeta = 0.7;
wn   = 4.0;
M    = wn^2 / (s^2 + 2*zeta*wn*s + wn^2);

% Ideal prefilter (may be non-proper)
Q_ideal = minreal(M * (1 + L) / G);

% Approximate Q(s) by a first-order model with unit DC gain
Tq = 0.2;
Q = 1 / (Tq * s + 1);

% 2-DOF tracking transfer function
Tyr = minreal(G * Q / (1 + L));

figure; step(Tyr); grid on;
title('2-DOF tracking with prefilter Q(s)');

figure; step(Tyd); grid on;
title('Disturbance response T_{yd}(s) (unchanged by Q)');
