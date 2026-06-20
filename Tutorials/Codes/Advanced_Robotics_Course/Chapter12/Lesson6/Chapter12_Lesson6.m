% Observation and action specifications
obsDim = 6; % [q1, q2, dq1, dq2, ee_x - goal_x, ee_y - goal_y]
actDim = 2;

obsInfo = rlNumericSpec([obsDim 1], ...
    'LowerLimit', -ones(obsDim,1)*inf, ...
    'UpperLimit',  ones(obsDim,1)*inf);
obsInfo.Name = 'observations';

actInfo = rlNumericSpec([actDim 1], ...
    'LowerLimit', -5*ones(actDim,1), ...
    'UpperLimit',  5*ones(actDim,1));
actInfo.Name = 'torques';

% Create Simulink environment
mdl = 'TwoLinkRLModel';
agentBlk = [mdl '/RL Agent'];
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);

% Actor and critic networks (SAC-style)
statePath = [
    featureInputLayer(obsDim, 'Normalization', 'none', 'Name', 'obs')
    fullyConnectedLayer(256, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(256, 'Name', 'fc2')
    reluLayer('Name', 'relu2')];

criticHead = [
    fullyConnectedLayer(1, 'Name', 'value')];

criticNet = layerGraph(statePath);
criticNet = addLayers(criticNet, criticHead);
criticNet = connectLayers(criticNet, 'relu2', 'value');

actorNet = layerGraph([
    featureInputLayer(obsDim, 'Normalization', 'none', 'Name', 'obs')
    fullyConnectedLayer(256, 'Name', 'afc1')
    reluLayer('Name', 'arelu1')
    fullyConnectedLayer(256, 'Name', 'afc2')
    reluLayer('Name', 'arelu2')
    fullyConnectedLayer(actDim, 'Name', 'mean')]);

criticOpts = rlOptimizerOptions('LearnRate', 1e-3, 'GradientThreshold', 1.0);
actorOpts  = rlOptimizerOptions('LearnRate', 1e-3, 'GradientThreshold', 1.0);

critic = rlValueFunction(criticNet, obsInfo, ...
    'ObservationInputNames', 'obs', ...
    'OptimizerOptions', criticOpts);

actor = rlContinuousGaussianActor(actorNet, obsInfo, actInfo, ...
    'ObservationInputNames', 'obs', ...
    'ActionMeanOutputNames', 'mean', ...
    'OptimizerOptions', actorOpts);

agentOpts = rlSACAgentOptions( ...
    'SampleTime', 0.02, ...
    'ExperienceBufferLength', 1e6, ...
    'DiscountFactor', 0.99);

agent = rlSACAgent(actor, critic, agentOpts);

% Train in simulation
trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', 500, ...
    'MaxStepsPerEpisode', 200, ...
    'ScoreAveragingWindowLength', 20, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', -1.0, ...
    'Verbose', true);

trainingStats = train(agent, env);

% Transfer test: change link masses in the Simulink model
set_param([mdl '/TwoLinkDynamics'], 'LinkMassScale', '1.3'); % e.g. mask parameter
simOptions = rlSimulationOptions('MaxSteps', 200);
simResults = sim(env, agent, simOptions);
      
