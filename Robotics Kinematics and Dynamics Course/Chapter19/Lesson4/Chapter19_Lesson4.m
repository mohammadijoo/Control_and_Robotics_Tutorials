function metrics = validate_dynamics_model(tau_meas, tau_pred, p_params)
% tau_meas, tau_pred: N-by-m matrices
% p_params: number of estimated dynamic parameters

[N, m] = size(tau_meas);
e = tau_meas - tau_pred;

% RMSE over all joints
rmse = sqrt(mean(sum(e.^2, 2)));

y = tau_meas(:);
y_hat = tau_pred(:);
e_flat = y - y_hat;

y_mean = mean(y);
e_mean = mean(e_flat);

var_y = var(y, 1);   % population form (1/N)
var_e = var(e_flat, 1);

vaf = 100 * (1 - var_e / var_y);

num = norm(y - y_hat);
den = norm(y - y_mean);
fit_percent = 100 * (1 - num / den);

tss = sum((y - y_mean).^2);
rss = sum((y - y_hat).^2);
r2 = 1 - rss / tss;

sigma2_hat = mean(e_flat.^2);
Nm = N * m;
aic = 2 * p_params + Nm * log(sigma2_hat);
bic = p_params * log(Nm) + Nm * log(sigma2_hat);

metrics = struct('rmse', rmse, ...
                 'vaf', vaf, ...
                 'fit_percent', fit_percent, ...
                 'r2', r2, ...
                 'aic', aic, ...
                 'bic', bic);
end
      
