% Monte Carlo evaluation of a controller or planner
N = 200;                % number of trials
success = false(N, 1);
cost = zeros(N, 1);

for k = 1:N
    % Set random seed or task parameters here, e.g.:
    % set_param('myModel', 'SimulationCommand', 'update');
    % assignin('base', 'initial_state', rand(3,1));

    simOut = sim('myAutonomyModel', 'ReturnWorkspaceOutputs', 'on');
    success(k) = simOut.success(end) > 0.5;
    cost(k) = simOut.cost(end);
end

p_hat = mean(success);
N_s = N;
alpha = 0.05;
z = 1.96;
se_p = sqrt(p_hat * (1 - p_hat) / N_s);
ci_p = [p_hat - z * se_p, p_hat + z * se_p];

c_bar = mean(cost);
s_c = std(cost, 1);
ci_c = [c_bar - z * s_c / sqrt(N), c_bar + z * s_c / sqrt(N)];

fprintf('Success rate: %.3f, CI = [%.3f, %.3f]\n', p_hat, ci_p(1), ci_p(2));
fprintf('Mean cost   : %.3f, CI = [%.3f, %.3f]\n', c_bar, ci_c(1), ci_c(2));
      
