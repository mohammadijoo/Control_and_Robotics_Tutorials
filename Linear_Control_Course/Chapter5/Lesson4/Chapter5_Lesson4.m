K = 2.0;
tau = 0.1;

% First-order transfer function
s = tf('s');
G = K / (tau*s + 1);

% Step response
t = 0:0.001:1.0;
[y, t_out] = step(G, t);

% Performance metrics
t_delay = -tau * log(0.5);
t_rise  = tau * log(9.0);    % 10-90%
t_settle_2 = -tau * log(0.02);

fprintf('t_d   = %f s\n', t_delay);
fprintf('t_r   = %f s\n', t_rise);
fprintf('t_s2%% = %f s\n', t_settle_2);

figure;
plot(t_out, y); hold on;
yline(K, '--', 'Final value');
xline(t_delay, ':', 't_d');
xline(t_rise, ':', 't_r');
xline(t_settle_2, ':', 't_s 2%%');
grid on;
xlabel('t [s]');
ylabel('y(t)');

% Simulink implementation:
% - Use a "Step" block with amplitude 1.
% - Use a "Transfer Fcn" block with numerator [K] and denominator [tau 1].
% - Use "Scope" or "To Workspace" blocks to visualize y(t).
