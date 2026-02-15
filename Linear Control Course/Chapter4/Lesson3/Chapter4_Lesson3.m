% Parameters
J   = 0.02;
B   = 0.1;
Km  = 0.5;
Kp  = 20.0;
Kenc = 1.0;

% Transfer functions
s  = tf('s');
Gp = Km / (J*s^2 + B*s);
Gc = Kp;
H  = Kenc;

% Closed-loop transfer function T(s) = feedback(Gc*Gp, H)
G = series(Gc, Gp);         % Gc(s)*Gp(s)
T = feedback(G, H);         % Negative feedback by default

step(T); grid on;
title('Closed-loop step response: position control');

% In Simulink, the same loop is drawn with blocks:
% 1) Use "Transfer Fcn" block for Gp(s)
% 2) Use "Gain" block for Kp
% 3) Use "Sum" block for error e = r - y_fb
% 4) Use "Gain" for H(s) or connect output directly for unity feedback
