% Robot joint plant: G(s) = K / ((T_m s + 1) s)
K  = 50;
Tm = 0.05;
s  = tf('s');
G  = K / ((Tm * s + 1) * s);

% Feedback-side PID
Kp = 20;
Ki = 200;
Kd = 0.02;

Cy = Kp + Ki / s + Kd * s;

% 2-DOF weights
b = 0.3;
c = 0.0;

Cr = Kp * b + Ki / s + Kd * c * s;

% Closed-loop from reference and from disturbance
Tr = minreal(G * Cr / (1 + G * Cy));
Td = minreal(G / (1 + G * Cy));

t = linspace(0, 0.5, 2000);
figure; step(Tr, t); grid on;
title('2-DOF PID: response to reference step');

figure; step(-Td, t); grid on;
title('2-DOF PID: response to disturbance step');

% In Simulink:
% - Use a PID Controller block configured for 2-DOF (if available),
%   or implement b and c weighting using Sum and Gain blocks.
% - Place a prefilter block F(s) in front of the reference if needed.
