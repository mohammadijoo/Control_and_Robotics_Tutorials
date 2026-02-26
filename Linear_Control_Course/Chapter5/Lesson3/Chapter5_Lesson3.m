K = 1.0;
tau = 0.4;

% Define first-order transfer function G(s) = K / (tau s + 1)
s = tf('s');
G = K / (tau * s + 1);

% Step response and highlighting tau
t = 0:0.01:5*tau;
[y, t_out] = step(G, t);
figure;
plot(t_out, y, 'LineWidth', 1.5); hold on;
yline(K, '--');
xline(tau, ':');
xlabel('t [s]');
ylabel('y(t)');
title('First-order step response');
grid on;

% In Simulink:
% - Place a Transfer Fcn block with numerator [K] and denominator [tau 1].
% - Drive it by a Step block.
% - Use a Scope to visualize y(t).
