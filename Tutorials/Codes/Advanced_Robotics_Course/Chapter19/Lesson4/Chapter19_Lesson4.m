% State x = [position; velocity], input u = acceleration
A = [1, 0.05;
     0, 1];
B = [0.5 * 0.05^2;
     0.05];
C = eye(2);
D = zeros(2, 1);

sys = ss(A, B, C, D, 0.05);   % discrete-time state-space model

% Define MPC object using this world model
predictionHorizon = 20;
controlHorizon = 5;
mpcobj = mpc(sys, 0.05, predictionHorizon, controlHorizon);

% Weights on output tracking and input effort
mpcobj.Weights.OutputVariables = [1, 0.1];
mpcobj.Weights.ManipulatedVariablesRate = 0.01;

% Simulate closed-loop control toward reference position x_star
x0 = [0; 0];
yref = [1; 0];   % target position 1, zero velocity
v = [];          % no measured disturbance
options = mpcsimopt(mpcobj);
options.PlantInitialState = x0;

[y, t, u] = sim(mpcobj, 60, yref, v, options);

% y contains imagined states under the world model, u the control sequence
figure;
subplot(2,1,1);
plot(t, y(:,1));
xlabel('time step');
ylabel('position');
subplot(2,1,2);
plot(t, u);
xlabel('time step');
ylabel('acceleration');
      
