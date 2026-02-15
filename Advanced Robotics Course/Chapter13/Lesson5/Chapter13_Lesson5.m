% Randomized transfer evaluation for a Simulink model "RandomizedArm"
% The model should read parameters: massScale, frictionScale, latency

rng(42);

N = 40;               % number of evaluation parameter sets
maxTime = 5.0;        % seconds per simulation
successThreshold = 0.95;  % fraction of time within target tolerance

massRange = [0.9, 1.1];
fricRange = [0.8, 1.2];
latRange  = [0.0, 0.03];

J_sim  = zeros(N,1);
J_real = zeros(N,1);
S_sim  = zeros(N,1);
S_real = zeros(N,1);

for i = 1:N
    massScale = massRange(1) + rand() * (massRange(2) - massRange(1));
    frictionScale = fricRange(1) + rand() * (fricRange(2) - fricRange(1));
    latency = latRange(1) + rand() * (latRange(2) - latRange(1));

    % Simulated environment
    assignin('base', 'massScale', massScale);
    assignin('base', 'frictionScale', frictionScale);
    assignin('base', 'latency', latency);

    simOut = sim('RandomizedArm', 'StopTime', num2str(maxTime), ...
        'SaveOutput', 'on', 'SaveState', 'off');

    rewardSignal = simOut.logsout.get('reward').Values.Data;
    successSignal = simOut.logsout.get('is_success').Values.Data;

    J_sim(i) = trapz(rewardSignal);
    S_sim(i) = max(successSignal);   % 1 if success at any time

    % Real robot experiment handler (returns same fields)
    realOut = runRealRobotExperiment(massScale, frictionScale, latency, maxTime);
    J_real(i) = trapz(realOut.reward);
    S_real(i) = max(realOut.is_success);
end

% Compute means and transfer gaps
J_sim_mean  = mean(J_sim);
J_real_mean = mean(J_real);
p_sim       = mean(S_sim);
p_real      = mean(S_real);

gap_return  = J_real_mean - J_sim_mean;
gap_success = p_real - p_sim;

fprintf('Expected return (sim, real): %.3f, %.3f\n', J_sim_mean, J_real_mean);
fprintf('Success prob. (sim, real): %.3f, %.3f\n', p_sim, p_real);
fprintf('Transfer gaps (return, success): %.3f, %.3f\n', gap_return, gap_success);
      
