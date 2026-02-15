function results = evaluate_autonomy_stack(modelName, N, horizon, Ts)
% modelName: name of Simulink model
% N: number of Monte Carlo runs
% horizon: number of steps
% Ts: sampling time

J = zeros(N,1);
succ = zeros(N,1);
viol = zeros(N,1);

for i = 1:N
    rng(i);  % deterministic noise seed

    % Configure initial conditions and parameters for this run
    omega.init_state = randn(6,1);
    omega.goal = [1; 0; 0; 0; 0; 0];

    % For a more sophisticated setup, use Simulink.SimulationInput
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setVariable('omega', omega);
    simIn = simIn.setVariable('horizon', horizon);
    simIn = simIn.setVariable('Ts', Ts);

    simOut = sim(simIn);

    cost_signal = simOut.logsout.get('cost');
    J(i) = sum(cost_signal.Values.Data);

    coll_signal = simOut.logsout.get('collision');
    violated = any(coll_signal.Values.Data > 0.5);
    viol(i) = double(violated);

    success_signal = simOut.logsout.get('task_success');
    done_signal = simOut.logsout.get('done');
    if any(done_signal.Values.Data > 0.5) && ~violated
        succ(i) = double(any(success_signal.Values.Data > 0.5));
    else
        succ(i) = 0;
    end
end

results.J_hat = mean(J);
results.J_std_over_sqrtN = std(J, 1) / sqrt(N);
results.p_succ_hat = mean(succ);
results.p_succ_std_over_sqrtN = std(succ, 1) / sqrt(N);
results.p_viol_hat = mean(viol);
results.p_viol_std_over_sqrtN = std(viol, 1) / sqrt(N);
      
