% MATLAB script
s = tf('s');

G  = 1 / (0.5*s + 1);      % Plant
Gd = 0.2 / (0.2*s + 1);    % Disturbance path

% PI feedback controller
kp = 8; ki = 4;
C  = kp + ki/s;

% Ideal disturbance feedforward and implementable approximation
F_ideal = -Gd / G;         % May be improper
F_ideal = minreal(F_ideal);

% Add roll-off pole if needed for realizability
if ~isproper(F_ideal)
    alpha = 20;
    rolloff = 1 / (s/alpha + 1);
    F = minreal(F_ideal * rolloff);
else
    F = F_ideal;
end

Tyw = minreal((G*F + Gd) / (1 + G*C)); % Disturbance->output transfer
step(Tyw);
grid on;
title('Step Response from Disturbance w to Output y with Feedforward');
