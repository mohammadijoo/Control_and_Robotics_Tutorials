% Chapter6_Lesson4.m
% Independence Assumptions and Their Limits — correlated-sensor Bayesian fusion (MATLAB)
%
% Demonstrates overconfidence when correlation is ignored.
% Requires: base MATLAB. Optional: Robotics System Toolbox for later integration.
%
% Run:
%   Chapter6_Lesson4

clear; clc;

mu0 = 0.0;
sigma0 = 2.0;
sigma = 1.0;
y = [1.0; 1.2];

fprintf('Prior: x ~ N(mu0, sigma0^2), mu0=%.6f, sigma0=%.6f\n', mu0, sigma0);
fprintf('Measurements: y1=%.6f, y2=%.6f, sigma=%.6f\n\n', y(1), y(2), sigma);

rhos = [0.0, 0.3, 0.6, 0.9];
for rho = rhos
    [m_corr, v_corr] = posterior_correlated(mu0, sigma0, y, sigma, rho);
    [m_ind, v_ind]   = posterior_independent(mu0, sigma0, y, sigma);

    ratio = v_ind / v_corr;

    fprintf('rho=%.1f:\n', rho);
    fprintf('  Correct correlated posterior: mean=%.6f, var=%.6f\n', m_corr, v_corr);
    fprintf('  Indep. assumption posterior:  mean=%.6f, var=%.6f\n', m_ind, v_ind);
    fprintf('  Variance ratio (indep/correct) = %.6f ( < 1 means overconfident )\n\n', ratio);
end

% Optional visualization
try
    rho_grid = linspace(0, 0.95, 25);
    v_corrs = zeros(size(rho_grid));
    v_inds  = zeros(size(rho_grid));
    for k = 1:numel(rho_grid)
        [~, v_corrs(k)] = posterior_correlated(mu0, sigma0, y, sigma, rho_grid(k));
        [~, v_inds(k)]  = posterior_independent(mu0, sigma0, y, sigma);
    end
    figure; 
    plot(rho_grid, v_corrs, 'LineWidth', 1.5); hold on;
    plot(rho_grid, v_inds,  'LineWidth', 1.5);
    xlabel('Correlation \rho'); ylabel('Posterior variance');
    title('Effect of ignoring correlation on posterior variance');
    legend('Correct correlated', 'Assume independent (\rho=0)');
    grid on;
catch ME
    disp('Plot skipped:'); disp(ME.message);
end

% ---------- Local functions ----------
function [post_mean, post_var] = posterior_correlated(mu0, sigma0, y, sigma, rho)
    Sigma = (sigma^2) * [1, rho; rho, 1];
    Sinv = inv(Sigma);
    H = [1; 1];
    info_meas = H' * Sinv * H;
    post_var = 1 / (1/(sigma0^2) + info_meas);
    info_vec = H' * Sinv * y;
    post_mean = post_var * (mu0/(sigma0^2) + info_vec);
end

function [post_mean, post_var] = posterior_independent(mu0, sigma0, y, sigma)
    post_var = 1 / (1/(sigma0^2) + 2/(sigma^2));
    post_mean = post_var * (mu0/(sigma0^2) + sum(y)/(sigma^2));
end

% ---------- Simulink note ----------
% To embed this in Simulink, you can create a MATLAB Function block "Fuse2Sensors"
% that implements posterior_independent or posterior_correlated, and feed it
% with y1,y2 constants (or from sensors). Programmatic construction uses:
%   new_system('Ch6L4_IndepAssumptions'); open_system('Ch6L4_IndepAssumptions');
%   add_block('simulink/Sources/Constant', ...); add_block('simulink/User-Defined Functions/MATLAB Function', ...);
%   add_line(...); save_system(...);
