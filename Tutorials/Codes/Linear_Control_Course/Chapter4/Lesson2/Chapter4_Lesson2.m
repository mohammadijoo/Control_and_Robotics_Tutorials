% Parameters
J  = 0.01;
B  = 0.02;
K  = 1.0;
Kt = 0.5;

s = tf('s');
G = Kt / (J*s^2 + B*s + K);

[zG, pG, kG] = zpkdata(G, 'v');  % zeros, poles, gain in vector form
disp('Poles:'); disp(pG);
disp('Zeros:'); disp(zG);

figure; pzmap(G); grid on;
title('Pole-zero map of joint model G(s)');

% In Simulink, one would:
% 1) Use a Transfer Fcn block with numerator [Kt] and denominator [J B K]
% 2) Drive it with a Step block (torque command)
% 3) Observe joint angle with a Scope block
