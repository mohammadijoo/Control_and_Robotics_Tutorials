% Define Laplace variable and basic transfer functions
s = tf('s');

% Simple robotic joint: Gp(s) = K / (J s^2 + b s)
J = 0.01;
b = 0.1;
K = 1.0;
Gp = K / (J*s^2 + b*s);

% Controller: proportional gain
Kc = 20;
Gc = Kc;

% Sensor gain
Ks = 1;
H = Ks;

% Series connection: controller then plant
Gforward = series(Gc, Gp);  % or simply: Gforward = Gc*Gp;

% Closed-loop from reference to output (negative feedback)
T = feedback(Gforward, H);  % shorthand for Gforward / (1 + Gforward*H)

% Display transfer function and step response
T = minreal(T);  % cancel any common factors
disp('Closed-loop transfer function T(s) = Y(s)/R(s):');
T

figure;
step(T);
title('Closed-loop step response of joint position');

% In Simulink, an equivalent model can be built using blocks:
%  - 'Gain' blocks for Kc and Ks
%  - 'Transfer Fcn' block for Gp(s)
%  - 'Sum' blocks for summing junctions
%  - 'Scope' for visualizing Y(t)
% and the reduction is implicit in the underlying simulation.
