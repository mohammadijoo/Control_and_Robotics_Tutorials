function stress_test_matlab()
    N = 5000;
    delta = 0.05;

    failures = 0;

    for i = 1:N
        [friction, mass, latency] = sample_scenario();
        [failed, label] = simulate_trial(friction, mass, latency); %#ok<NASGU>
        if failed
            failures = failures + 1;
            % Use containers.Map to maintain taxonomy counts if desired.
        end
    end

    p_hat = failures / N;
    eps = sqrt(log(2.0 / delta) / (2.0 * N));
    ci_low = max(0.0, p_hat - eps);
    ci_high = min(1.0, p_hat + eps);

    fprintf('N = %d\n', N);
    fprintf('Estimated failure probability: %.4f\n', p_hat);
    fprintf('Hoeffding 95%% CI: [%.4f, %.4f]\n', ci_low, ci_high);
end

function [friction, mass, latency] = sample_scenario()
    friction = betarnd(0.5, 2.0);   % requires Statistics and Machine Learning Toolbox
    mass = lognrnd(0.0, 0.25);
    latency = exprnd(0.08);
end

function [failed, label] = simulate_trial(friction, mass, latency)
    stopping_violation = (friction * mass) < 0.6;
    latency_violation = latency > 0.15;
    timeout_violation = (mass > 2.5) && (latency > 0.10);

    if stopping_violation && latency_violation
        failed = true;
        label = 'control+environment:multi-factor';
    elseif stopping_violation
        failed = true;
        label = 'control:insufficient_braking';
    elseif latency_violation
        failed = true;
        label = 'control:latency_instability';
    elseif timeout_violation
        failed = true;
        label = 'performance:timeout';
    else
        failed = false;
        label = 'success';
    end

    % In a Simulink workflow, this function would:
    %  - configure model parameters (friction, mass, latency),
    %  - run sim('model') to obtain trajectories,
    %  - compute constraint violations and taxonomy labels from logged signals.
end
      
