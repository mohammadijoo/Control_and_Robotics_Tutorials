% Assume you have a Simulink model "planner_benchmark"
% that, for each run, writes results to the workspace variables:
%   success (logical), pathLength (double), solveTime (double).

nTrials = 200;
success = false(nTrials,1);
pathLength = nan(nTrials,1);
solveTime = nan(nTrials,1);

for k = 1:nTrials
    % Randomize initial condition and goal
    q0 = rand(1,6) * 2*pi - pi;
    qg = rand(1,6) * 2*pi - pi;
    assignin('base','q0',q0);
    assignin('base','qg',qg);

    % Run Simulink model once
    simOut = sim('planner_benchmark','StopTime','10');

    success(k)    = simOut.success(end);
    pathLength(k) = simOut.pathLength(end);
    solveTime(k)  = simOut.solveTime(end);
end

% Success rate and CI
pHat = mean(success);
alpha = 0.05;
z = norminv(1 - alpha/2);
n = nTrials;
halfWidthSucc = z * sqrt(pHat * (1 - pHat) / n);
succCI = [pHat - halfWidthSucc, pHat + halfWidthSucc];

% Mean path length and CI over successful runs
idx = success == 1;
L = pathLength(idx);
muL = mean(L);
sigmaL = std(L,1);          % population std
seL = sigmaL / sqrt(numel(L));
halfWidthL = z * seL;
lenCI = [muL - halfWidthL, muL + halfWidthL];

fprintf('Success rate: %.3f [%.3f, %.3f]\n', pHat, succCI(1), succCI(2));
fprintf('Path length (success): %.3f [%.3f, %.3f]\n', muL, lenCI(1), lenCI(2));
      
