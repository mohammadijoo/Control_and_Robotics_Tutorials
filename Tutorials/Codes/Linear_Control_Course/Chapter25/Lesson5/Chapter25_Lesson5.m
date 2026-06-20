s = tf('s');

% Inner current loop (approximate electrical dynamics)
G_i = 1 / (1e-3 * s + 0.5);
C_i = 50 + 5000 / s;
T_i = feedback(C_i * G_i, 1);   % inner closed loop

% Speed plant (mechanical dynamics)
J = 0.01; B = 0.001; Kt = 0.05;
G_w = Kt / (J * s + B);

% Speed loop on top of closed current loop
C_w = 2 + 200 / s;
G_eq = T_i * G_w;
T_w = feedback(C_w * G_eq, 1);

figure;
step(T_w);
grid on;
title('Speed loop step response with inner current loop');

% In Simulink, the same structure is built by nesting Feedback blocks:
% outer speed PID feeding the reference of an inner current PID,
% which drives a Transfer Fcn block for G_i and a mechanical plant block.
