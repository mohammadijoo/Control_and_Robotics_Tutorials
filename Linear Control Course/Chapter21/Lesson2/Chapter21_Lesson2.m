% Plant parameter
tau = 0.1;

% Desired bandwidth (rad/s)
omega_b_target = 20;

% Proportional gain
K = tau * omega_b_target - 1;

% Plant and controller
G = tf(1, [tau 1]);
C = K;

% Closed-loop transfer function
T = feedback(C * G, 1);

% MATLAB's built-in bandwidth function
[omega_b_est, ~] = bandwidth(T);

fprintf("Designed K = %.3f\n", K);
fprintf("Estimated bandwidth omega_b ≈ %.2f rad/s\n", omega_b_est);

% In Simulink:
%  - Use a Transfer Fcn block with numerator [1] and denominator [tau 1].
%  - Use a Gain block with value K.
%  - Connect them in a unity feedback loop and verify bandwidth with Bode and step plots.
