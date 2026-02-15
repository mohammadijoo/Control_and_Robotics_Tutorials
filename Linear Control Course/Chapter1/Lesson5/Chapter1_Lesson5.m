% Time vector
t = linspace(0, 10, 1001);
u = ones(size(t));  % unit step

% LTI system: dy/dt + y = u
f_lti = @(t, y, u_val) (-y + u_val);

% LTV system: dy/dt + (1 + 0.5*t)*y = u
f_ltv = @(t, y, u_val) (-(1 + 0.5*t)*y + u_val);

% Simple forward Euler simulation
dt = t(2) - t(1);
y_lti = zeros(size(t));
y_ltv = zeros(size(t));

for k = 1:length(t)-1
    y_lti(k+1) = y_lti(k) + dt * f_lti(t(k), y_lti(k), u(k));
    y_ltv(k+1) = y_ltv(k) + dt * f_ltv(t(k), y_ltv(k), u(k));
end

figure;
plot(t, y_lti, 'LineWidth', 1.5); hold on;
plot(t, y_ltv, '--', 'LineWidth', 1.5);
xlabel('t [s]');
ylabel('y(t)');
legend('LTI', 'LTV');
grid on;
title('Response of LTI vs LTV first-order systems');

% In Simulink, one would construct equivalent diagrams using Integrator blocks
% and Gain blocks where the LTV coefficient is implemented as a time-varying block.
