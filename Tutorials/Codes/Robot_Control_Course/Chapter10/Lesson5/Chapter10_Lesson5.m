
% Joint-space double-integrator MPC in MATLAB
n  = 2;
Ts = 0.02;

I = eye(n);
A = [I, Ts * I;
     zeros(n), I];
B = [zeros(n);
     Ts * I];
C = [eye(n), zeros(n)];   % measure joint error (here we treat x directly)
D = zeros(n, n);

plant = ss(A, B, C, D, Ts);

% Weights and horizon
Q = diag([100, 100, 10, 10]);
R = 0.1 * eye(n);
P = Q;  % for simplicity

mpc_horizon = 20;
control_horizon = 3;

mpcobj = mpc(plant, Ts, mpc_horizon, control_horizon);

% Set weights on states and inputs (Toolbox uses output/reference weights)
mpcobj.Weights.OutputVariables = [1 1];     % position error
mpcobj.Weights.ManipulatedVariables = [0.1 0.1];
mpcobj.Weights.ManipulatedVariablesRate = [0.0 0.0];

% Input (virtual acceleration) constraints
mpcobj.MV(1).Min = -5;
mpcobj.MV(1).Max =  5;
mpcobj.MV(2).Min = -5;
mpcobj.MV(2).Max =  5;

% In Simulink:
% 1. Create a Simulink model with a "State-Space" block representing the plant.
% 2. Add an "MPC Controller" block and link it to mpcobj (or use "mpcDesigner").
% 3. Connect measured states (or outputs) and reference commands.
% 4. Run the simulation and visualize joint tracking.

% Direct simulation example with sim():
x0 = [0.2; -0.1; 0; 0];
Tf = 4.0;
sim(mpcobj, Tf / Ts, x0);
