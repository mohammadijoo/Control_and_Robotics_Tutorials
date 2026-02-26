% Plant and design parameters
T = 0.5;
zeta = 0.7;
ts_des = 1.0;
wn = 4 / (zeta * ts_des);

Kp = 0.5 * wn^2;
Td = (zeta * wn - 1) / Kp;

s = tf('s');

G = 1 / (s * (T*s + 1));     % DC motor position plant
C = Kp * (1 + Td * s);       % PD controller

L = C * G;
Tcl = feedback(L, 1);

% Step response
figure;
step(Tcl);
grid on;
title('Closed-loop step response (MATLAB)');

% Sensitivity and complementary sensitivity
S = feedback(1, L);
Tmat = feedback(L, 1);

figure;
bodemag(S, Tmat);
grid on;
legend('S(s)', 'T(s)');
title('Sensitivity and complementary sensitivity');

% Simulink hint:
% - Create a new model.
% - Add a Step block for reference r(t).
% - Add Sum block for error e(t) = r - theta.
% - Implement C(s) as a Transfer Fcn or as series of Gain and Derivative blocks.
% - Add Transfer Fcn block for G(s).
% - Close the loop with a second Sum block.
% - Use Scope block to visualize theta(t) and e(t).
