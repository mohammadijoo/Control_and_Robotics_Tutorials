% Parameters
Kp    = 2.0;
tau_p = 0.1;
Kc    = 5.0;

% Plant and controller
s = tf('s');
P = Kp / (tau_p * s + 1);
C = Kc;

% Loop transfer and complementary sensitivity
L = C * P;
T = feedback(L, 1);  % T(s) = L / (1 + L)

figure;
bode(T);
grid on;
title('Complementary Sensitivity T(s)');

% Time-domain simulation in Simulink (outline):
% 1. Create a Simulink model with:
%    - Sum block for error e = r - y_meas
%    - Gain block C(s) (for proportional control just a constant)
%    - Transfer Fcn block for P(s)
%    - Sum block to form y_meas = y + n
%    - Noise source: Band-limited White Noise block feeding the measurement
% 2. Connect the blocks according to the conceptual diagram.
% 3. Use a Step block for reference r(t) and Scope blocks to observe y(t) and u(t).
% 4. Use the 'sim' command from MATLAB to run parameter sweeps on Kc and observe
%    changes in noise level vs tracking speed.
