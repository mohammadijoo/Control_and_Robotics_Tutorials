% System: xdot = -x + u(t), y = x
u = @(t) double(t >= 0);   % unit step
f = @(t, x) -x + u(t);

tspan = [0 5];
x0 = 0.2;

[t, x] = ode45(f, tspan, x0);
y = x;

fprintf('x(tf) = %.6f\n', x(end));
fprintf('y(tf) = %.6f\n', y(end));

% Simulink (block-level description, no image):
% - Use an Integrator block for x
% - Feed back x through a Gain block of -1
% - Sum (-x) + u using a Sum block
% - Output y is taken as x
