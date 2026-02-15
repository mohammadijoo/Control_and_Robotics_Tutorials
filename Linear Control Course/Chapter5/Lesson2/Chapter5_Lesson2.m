% Parameters
K   = 2.0;
tau = 0.5;

% Transfer function G(s) = K / (tau s + 1)
s = tf('s');
G = K / (tau * s + 1);

t = linspace(0, 5, 1000);

% Step response (unit step)
[y_step, t_step] = step(G, t);

% Ramp response: input r(t) = t * u(t)
u_ramp = t;  % unit ramp
[y_ramp, t_ramp, x_ramp] = lsim(G, u_ramp, t);

% Impulse response
[y_imp, t_imp] = impulse(G, t);

% Plot
figure;
plot(t_step, y_step, 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('y_{step}(t)');
title('First-Order Step Response'); grid on;

figure;
plot(t_ramp, y_ramp, 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('y_{ramp}(t)');
title('First-Order Ramp Response'); grid on;

figure;
plot(t_imp, y_imp, 'LineWidth', 1.5);
xlabel('t [s]'); ylabel('y_{imp}(t)');
title('First-Order Impulse Response'); grid on;

% In Simulink:
% - Use a Transfer Fcn block with numerator K and denominator [tau 1].
% - Drive it with Step, Ramp, and Pulse Generator sources.
% - Connect to Scope blocks to visualize the responses.
% With Robotics System Toolbox, these blocks can be integrated with robot models.
