% Build regressor Y for 1-DOF pendulum
Y = [qdd, sin(q), qd];   % size N-by-3
T = tau;                 % size N-by-1

% Ordinary least squares estimate
phi_hat = Y \ T;         % 3-by-1 vector [a1; a2; a3]

% Predicted torque and residual analysis
tau_pred = Y * phi_hat;
residual = tau - tau_pred;
rmse = sqrt(mean(residual.^2));
disp('Estimated parameters [a1 a2 a3]^T:');
disp(phi_hat);
disp(['RMSE: ', num2str(rmse)]);
      
