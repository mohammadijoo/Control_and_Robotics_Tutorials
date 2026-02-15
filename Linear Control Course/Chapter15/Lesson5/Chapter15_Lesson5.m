% Third-order plant for a robot joint:
% G(s) = 1 / (s (s + 1) (s + 3))
s = tf('s');
G = 1 / (s * (s + 1) * (s + 3));

% Proportional controller K
K = 9;
L = K * G;

figure;
nyquist(L);
hold on;
plot(-1, 0, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
title('Nyquist plot for L(s) = K G(s)');
xlabel('Real axis');
ylabel('Imag axis');

% Closed-loop transfer function
T = feedback(L, 1);
poles_T = pole(T)

% Simulink model (optional) to simulate step response in a robot joint
% new_system('nyquist_robot_joint');
% open_system('nyquist_robot_joint');
% add_block('simulink/Continuous/Transfer Fcn', ...
%           'nyquist_robot_joint/Plant');
% set_param('nyquist_robot_joint/Plant', 'Numerator', '1', ...
%           'Denominator', '[1 4 3 0]');
% add_block('simulink/Sources/Step', 'nyquist_robot_joint/Step');
% add_block('simulink/Math Operations/Gain', ...
%           'nyquist_robot_joint/Proportional Gain');
% set_param('nyquist_robot_joint/Proportional Gain', 'Gain', num2str(K));
% add_block('simulink/Sinks/Scope', 'nyquist_robot_joint/Scope');
% % Connect blocks to form unity-feedback loop and simulate.

% In Robotics System Toolbox, one could connect this axis model to a
% multibody robot model and reuse the same Nyquist-based gain K.
