function stats = evaluate_policy_sim(modelName, taskGenerator, N)
% modelName: name of Simulink model
% taskGenerator(i): returns struct of parameters for trial i

success = zeros(N,1);
timeVec = zeros(N,1);
energyVec = zeros(N,1);

for k = 1:N
    cfg = taskGenerator(k); % user-defined function returning config struct
    % Assign configuration to base workspace or model parameters
    assignin('base', 'taskConfig', cfg);

    simOut = sim(modelName, 'SaveOutput', 'on', 'SaveState', 'off');
    logs = simOut.logsout;
    % Assume logs contains signals "success", "episodeTime", "energy"
    success(k) = logs.get('success').Values.Data(end);
    timeVec(k) = logs.get('episodeTime').Values.Data(end);
    energyVec(k) = logs.get('energy').Values.Data(end);
end

p_hat = mean(success);
z = 1.96;
se_p = sqrt(p_hat*(1-p_hat)/N);
stats.p_hat = p_hat;
stats.p_ci = [max(0, p_hat - z*se_p), min(1, p_hat + z*se_p)];

stats.time_mean = mean(timeVec);
stats.time_std = std(timeVec, 1);    % population or sample as desired
stats.energy_mean = mean(energyVec);
stats.energy_std = std(energyVec, 1);
end
      
