% First-order transfer function G(s) = K / (tau s + 1)
K   = 2;
tau = 0.5;

s = tf('s');
G = K / (tau * s + 1);

disp('Transfer function G(s) = ');
G

% Unit-step response
figure;
step(G);
grid on;
title('Unit-step response of first-order system');

% In robotics, such a block can represent a simplified actuator model
% inside a joint servo loop (e.g., in Robotics System Toolbox-based simulations).

% In Simulink:
%  - Place a Transfer Fcn block with Numerator [K] and Denominator [tau 1].
%  - Connect a Step block to its input and a Scope to its output.
