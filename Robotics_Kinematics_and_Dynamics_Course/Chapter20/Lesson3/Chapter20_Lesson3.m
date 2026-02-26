% Parameter mean and covariance
theta_mu     = [1.5; 0.8];
Sigma_theta  = [0.04 0.0;
                0.0  0.01];

% Operating point
q   = 0.5;
qd  = 0.0;
qdd = 2.0;

% Regressor for 1-DOF system
Y = [qdd, sin(q)];   % 1-by-2

% Analytical mean and variance
tau_mu  = Y * theta_mu;
tau_var = Y * Sigma_theta * Y.';  % 1-by-1

fprintf("Analytical E[tau] = %f\n", tau_mu);
fprintf("Analytical Var[tau] = %f\n", tau_var);

% Monte Carlo sampling
Nmc = 100000;
theta_samples = mvnrnd(theta_mu.', Sigma_theta, Nmc);   % Nmc-by-2
tau_samples   = (Y * theta_samples.').';               % Nmc-by-1

fprintf("MC mean approx = %f\n", mean(tau_samples));
fprintf("MC var approx  = %f\n", var(tau_samples, 1));

% Optional: export samples for use in Simulink
tau_time = [(0:Nmc-1).', tau_samples];  % time index, value
save("tau_samples.mat", "tau_time");
      
