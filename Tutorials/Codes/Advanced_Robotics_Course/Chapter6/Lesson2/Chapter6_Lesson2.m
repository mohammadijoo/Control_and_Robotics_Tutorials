function [mu_next, sigma2_next] = belief_step_1d(mu, sigma2, u, z, sigma_w2, sigma_v2)
% One-step 1D Gaussian belief update for linear sensor.
% Prediction
mu_pred = mu + u;
sigma2_pred = sigma2 + sigma_w2;

% Update
K = sigma2_pred / (sigma2_pred + sigma_v2);
mu_next = mu_pred + K * (z - mu_pred);
sigma2_next = (1.0 - K) * sigma2_pred;
end

% Example script
mu = 0.0;
sigma2 = 1.0;
x_goal = 5.0;
sigma_w2 = 0.1;
sigma_v2 = 0.2;
lambda_unc = 0.5;
actions = [-1.0, 0.0, 1.0];

best_u = actions(1);
best_cost = inf;

for k = 1:numel(actions)
    u = actions(k);
    mu_pred = mu + u;
    sigma2_pred = sigma2 + sigma_w2;
    mu_err = mu_pred - x_goal;
    cost = mu_err^2 + lambda_unc * sigma2_pred;
    if cost < best_cost
        best_cost = cost;
        best_u = u;
    end
end

fprintf("Chosen control: %.2f\n", best_u);
      
