% Define observation and action spaces
obsInfo = rlNumericSpec([obsDim 1], ...
    'LowerLimit', -inf, 'UpperLimit', inf);
actInfo = rlNumericSpec([actDim 1], ...
    'LowerLimit', -uMax, 'UpperLimit', uMax);

% Create Simulink environment
mdl = "RobotPendulumModel";
open_system(mdl);
env = rlSimulinkEnv(mdl, "RL_Agent_Block", obsInfo, actInfo);

% Critic network (state-value)
statePath = [
    featureInputLayer(obsDim, "Normalization","none","Name","state")
    fullyConnectedLayer(64, "Name","fc1")
    reluLayer("Name","relu1")
    fullyConnectedLayer(64, "Name","fc2")
    reluLayer("Name","relu2")
    fullyConnectedLayer(1, "Name","value")
];
criticNet = dlnetwork(layerGraph(statePath));
critic = rlValueFunction(criticNet, obsInfo, ...
    "ObservationInputNames","state");

% Actor network (Gaussian mean)
actorPath = [
    featureInputLayer(obsDim, "Normalization","none","Name","state")
    fullyConnectedLayer(64, "Name","fc1")
    reluLayer("Name","relu1")
    fullyConnectedLayer(64, "Name","fc2")
    reluLayer("Name","relu2")
    fullyConnectedLayer(actDim, "Name","mu")
    tanhLayer("Name","tanh")
];
actorNet = dlnetwork(layerGraph(actorPath));
actor = rlContinuousGaussianActor(actorNet, obsInfo, actInfo, ...
    "ObservationInputNames","state");

agentOpts = rlACAgentOptions( ...
    "DiscountFactor", 0.99, ...
    "ActorOptimizerOptions", rlOptimizerOptions("LearnRate",1e-4), ...
    "CriticOptimizerOptions", rlOptimizerOptions("LearnRate",1e-3));

agent = rlACAgent(actor, critic, agentOpts);

% Train
trainOpts = rlTrainingOptions( ...
    "MaxEpisodes", 2000, ...
    "MaxStepsPerEpisode", 400, ...
    "StopTrainingCriteria","AverageReward", ...
    "StopTrainingValue",-50);

trainingStats = train(agent, env, trainOpts);
      
