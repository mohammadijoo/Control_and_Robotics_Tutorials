% Joint parameters
J = 0.01;
b = 0.1;
k = 1.0;

% ODE as first-order system: x(1) = q, x(2) = qd
joint_ode = @(t,x) [ ...
    x(2); ...
    (1.0 - b * x(2) - k * x(1)) / J ...
];

tspan = [0 5];
x0 = [0; 0];  % q(0) = 0, qd(0) = 0

[t, x] = ode45(joint_ode, tspan, x0);

figure;
plot(t, x(:,1), 'LineWidth', 1.5); hold on;
plot(t, x(:,2), 'LineWidth', 1.5);
xlabel('t [s]');
legend('q(t)', 'qd(t)');
title('Single-DOF joint response (MATLAB ode45)');
grid on;

% Minimal Simulink-style script (conceptual)
% 1) create a new model
% new_system('joint_model');
% open_system('joint_model');
%
% 2) add blocks (Integrator, Gain, Sum, Step, Scope) and connect them
% programmatically using add_block / add_line, or use the Simulink GUI.
% 3) parameterize gains with J, b, k and run:
% sim('joint_model');
      
