m = 1.0; b = 0.2; k = 1.0;

f = @(t, x) [x(2);
             (1.0 - b * x(2) - k * x(1)) / m];

[t, x] = ode45(f, [0 10], [0; 0]);  % [position; velocity]

plot(t, x(:, 1));
xlabel('t (s)');
ylabel('q(t)');
title('Mass-spring-damper step response');
grid on;

% In Simulink, the same dynamics are implemented with two Integrator blocks
% in series (for q and qdot) plus Sum and Gain blocks for the forces.
% Robotics System Toolbox represents manipulator dynamics via similar ODEs.
