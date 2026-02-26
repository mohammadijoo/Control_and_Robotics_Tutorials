function stats = evaluate_policy_simulink(modelName, numEpisodes)
% modelName: Simulink model implementing a task (e.g. long-horizon manipulation)
% The model should read policy parameters from the workspace and write:
%   is_success (logical), episode_cost (double)

if nargin < 2
    numEpisodes = 20;
end

totalSuccess = 0;
totalCost = 0.0;

for ep = 1:numEpisodes
    % Randomize initial conditions, goals, etc.
    % For example:
    % goalPose = randomSE3Pose();
    % assignin('base', 'goalPose', goalPose);

    % Run one episode
    simOut = sim(modelName, 'StopTime', 'Tfinal', ...
        'SaveOutput', 'on', 'SrcWorkspace', 'base');

    % Extract logged variables
    is_success = simOut.get('is_success');
    episode_cost = simOut.get('episode_cost');

    totalSuccess = totalSuccess + double(is_success(end) ~= 0);
    totalCost = totalCost + episode_cost(end);
end

stats.successRate = totalSuccess / numEpisodes;
stats.avgCost = totalCost / numEpisodes;
end
      
