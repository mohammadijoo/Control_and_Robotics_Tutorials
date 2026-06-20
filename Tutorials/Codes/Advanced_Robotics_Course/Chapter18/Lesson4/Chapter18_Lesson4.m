function run_repro_sim()
    % Configuration
    cfg.modelName   = 'two_link_manipulator_model';
    cfg.stopTime    = 5.0;
    cfg.globalSeed  = 1234;
    cfg.numRuns     = 30;
    cfg.logFile     = 'matlab_repro_results.mat';

    % Set global RNG for reproducibility
    rng(cfg.globalSeed, 'twister');

    % Configure model parameters (e.g., friction, payload)
    set_param(cfg.modelName, 'StopTime', num2str(cfg.stopTime));

    % Preallocate logs
    episodeCost = zeros(cfg.numRuns, 1);
    successFlag = false(cfg.numRuns, 1);

    for k = 1:cfg.numRuns
        % Optionally vary initial condition deterministically
        baseSeed = cfg.globalSeed + k - 1;
        rng(baseSeed, 'twister');

        % Set workspace variables used by the model
        assignin('base', 'initSeed', baseSeed);

        simOut = sim(cfg.modelName, ...
            'SimulationMode', 'normal', ...
            'SrcWorkspace', 'current', ...
            'SaveOutput', 'on', ...
            'SaveState', 'off');

        % Extract metrics (assuming logsout or out structure)
        logs = simOut.logsout;
        costSig = logs.get('cost');
        successSig = logs.get('success');

        episodeCost(k) = costSig.Values.Data(end);
        successFlag(k) = logical(successSig.Values.Data(end));
    end

    summary.meanCost    = mean(episodeCost);
    summary.stdCost     = std(episodeCost, 1);
    summary.successRate = mean(successFlag);

    save(cfg.logFile, 'cfg', 'episodeCost', 'successFlag', 'summary');
end
      
