% Parameters
tau = 0.05;          % time constant
wc  = 1/tau;         % cutoff frequency
s   = tf('s');

F_lp = wc / (s + wc);     % continuous-time low-pass filter

% Time-domain simulation on a noisy signal
dt = 0.001;
t  = 0:dt:1;
theta_true = 0.5 * t;
noise = 0.02 * randn(size(t));
y_meas = theta_true + noise;

y_filt = lsim(F_lp, y_meas, t);

% Plot
figure;
plot(t, y_meas, 'Color', [0.8 0.8 0.8]); hold on;
plot(t, y_filt, 'LineWidth', 1.5);
plot(t, theta_true, '--', 'LineWidth', 1.5);
legend('measured', 'filtered', 'true');
xlabel('t [s]');
ylabel('\theta [rad]');
title('First-order low-pass filter on encoder signal');

% In Simulink:
%  - Use a "Transfer Fcn" block with numerator [wc] and denominator [1 wc].
%  - Place this block in the measurement path before the controller input.
%  - Robotics System Toolbox can then connect the Simulink controller to a
%    robot model or a ROS network for full closed-loop simulations.
