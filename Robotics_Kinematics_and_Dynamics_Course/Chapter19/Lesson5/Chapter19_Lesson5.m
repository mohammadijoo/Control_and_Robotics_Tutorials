% Suppose we have vectors q, dq, ddq, tau_meas of equal length N
% (e.g., exported from a Simulink model of the joint).
N = numel(q);

% Build regressor for single-link example
Y = [ddq(:), cos(q(:)), dq(:)];    % N-by-3
tau_vec = tau_meas(:);            % N-by-1

% Remove a few samples from both ends to avoid differentiation artifacts
idx = 10:(N-10);
Y_id = Y(idx, :);
tau_id = tau_vec(idx);

% Least-squares estimate (normal equations)
pi_hat = (Y_id' * Y_id) \ (Y_id' * tau_id);

fprintf('Estimated parameters: pi_1 = %.4f, pi_2 = %.4f, pi_3 = %.4f\n', ...
        pi_hat(1), pi_hat(2), pi_hat(3));

% Validation on a different trajectory data set q_val, dq_val, ddq_val
Y_val = [ddq_val(:), cos(q_val(:)), dq_val(:)];
tau_pred = Y_val * pi_hat;
rms_val = sqrt(mean((tau_pred - tau_val(:)).^2));
fprintf('Validation RMS torque error: %.4f\n', rms_val);

% In a Simulink workflow:
% 1) Model the joint with Simscape Multibody, including inertia and friction.
% 2) Drive q(t) with a multi-sine excitation via a trajectory block.
% 3) Log q, dq, ddq, tau (e.g., from the joint actuator block).
% 4) Export the log to the workspace and run this identification script.
      
