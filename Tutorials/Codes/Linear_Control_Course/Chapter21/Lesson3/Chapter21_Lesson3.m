% Plant: P(s) = K_g / (T s + 1)
K_g = 2.0;
T = 0.5;
s = tf('s');
P = K_g / (T * s + 1);

% Integral controller: C(s) = K_I / s
K_I = 10.0;
C = K_I / s;

L = C * P;
T_cl = feedback(L, 1);  % unity-feedback closed loop

figure;
bode(L);
grid on;
title('Loop transfer function L(s) with integral action');

figure;
step(T_cl);
grid on;
title('Closed-loop step response with integral controller');

% In Simulink, the equivalent diagram uses:
% - "Sum" block for e(t) = r(t) - y(t)
% - "Integrator" block with gain K_I for C(s)
% - Transfer Fcn block for P(s)
% This controller can be connected to a robot joint model from Robotics System Toolbox.
