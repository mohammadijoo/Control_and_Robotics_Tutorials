K = 2.0;
T = 0.5;

% Transfer function G(s) = K / (T s + 1)
s = tf('s');
G = K / (T*s + 1)

% Step response (typical for servo systems)
figure;
step(G);
grid on;
title('Step response of first-order system');

% In Simulink, you would:
% 1. Create a new model.
% 2. Insert a Transfer Fcn block and set Numerator: [K], Denominator: [T 1].
% 3. Connect a Step block as input and a Scope block to visualize y(t).
