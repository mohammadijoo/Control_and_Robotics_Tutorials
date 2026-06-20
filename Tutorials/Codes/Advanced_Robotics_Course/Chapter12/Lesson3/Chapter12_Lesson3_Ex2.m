% Define observation and action specifications
obsDim = 14;  % joint angles + velocities
actDim = 7;   % joint torques

obsInfo = rlNumericSpec([obsDim 1], ...
    'LowerLimit', -ones(obsDim,1), ...
    'UpperLimit',  ones(obsDim,1));
obsInfo.Name = 'observations';

actInfo = rlNumericSpec([actDim 1], ...
    'LowerLimit', -ones(actDim,1), ...
    'UpperLimit',  ones(actDim,1));
actInfo.Name = 'torques';

% Create Simulink environment
mdl = 'RobotArmTD3Model';
load_system(mdl);
env = rlSimulinkEnv(mdl, 'RobotArmTD3Model/RL Agent', obsInfo, actInfo);

% Actor and critic networks (deep neural networks)
statePath = featureInputLayer(obsDim, 'Normalization', 'none', 'Name', 'state');
actionPath = featureInputLayer(actDim, 'Normalization', 'none', 'Name', 'action');

criticNetwork = [
    concatenationLayer(1, 2, 'Name', 'concat')
    fullyConnectedLayer(256, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(256, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(1, 'Name', 'q')];

criticGraph = layerGraph(criticNetwork);
criticGraph = addLayers(criticGraph, statePath);
criticGraph = addLayers(criticGraph, actionPath);
criticGraph = connectLayers(criticGraph, 'state', 'concat/in1');
criticGraph = connectLayers(criticGraph, 'action', 'concat/in2');

criticOpts = rlRepresentationOptions('LearnRate',1e-3,'GradientThreshold',1);
critic = rlQValueRepresentation(criticGraph, obsInfo, actInfo, ...
    'Observation', {'state'}, 'Action', {'action'}, criticOpts);

actorNetwork = [
    featureInputLayer(obsDim, 'Normalization', 'none', 'Name', 'state')
    fullyConnectedLayer(256, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(256, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(actDim, 'Name', 'fc3')
    tanhLayer('Name', 'tanh1')];

actorOpts = rlRepresentationOptions('LearnRate',1e-4,'GradientThreshold',1);
actor = rlDeterministicActorRepresentation(actorNetwork, obsInfo, actInfo, ...
    'Observation', {'state'}, 'Action', {'tanh1'}, actorOpts);

agentOpts = rlDDPGAgentOptions(...
    'SampleTime', 0.02, ...
    'TargetSmoothFactor', 5e-3, ...
    'DiscountFactor', 0.99, ...
    'MiniBatchSize', 256);

agent = rlDDPGAgent(actor, critic, agentOpts);

% Train the agent
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 1000, ...
    'MaxStepsPerEpisode', 1000, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 200);

trainingStats = train(agent, env, trainOpts);
      
