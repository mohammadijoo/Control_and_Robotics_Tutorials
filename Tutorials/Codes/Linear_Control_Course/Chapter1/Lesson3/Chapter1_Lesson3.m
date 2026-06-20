% Parameters
a  = 1.0;
b  = 1.0;
K  = 4.0;
r0 = 1.0;

% Closed-loop ODE:
% y_dot = -(a + b*K)*y + b*K*r0
odefun = @(t, y) -(a + b*K) * y + b * K * r0;

tspan = [0 3];
y0    = 0;

[t, y] = ode45(odefun, tspan, y0);

r = r0 * ones(size(t));

figure;
plot(t, r, '--', 'DisplayName', 'reference r(t)');
hold on;
plot(t, y, 'DisplayName', 'output y(t)');
xlabel('time [s]');
ylabel('signal');
legend;
title('First-order plant with proportional feedback');

% In Simulink, you would connect:
%  - A Sum block to compute e = r - y
%  - A Gain block (value K) for the controller
%  - A first-order plant block or an integrator implementing the ODE
% and feed back y to the Sum block.
