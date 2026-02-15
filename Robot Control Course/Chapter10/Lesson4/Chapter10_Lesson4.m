
% Discrete-time state-space model for joint-space dynamics
Ts = 0.01;
A = [1 Ts; 0 1];
B = [0; Ts];
C = [1 0];
D = 0;

plant = ss(A,B,C,D,Ts);

% MPC horizon and weights
p = 20;   % prediction horizon
m = 3;    % control horizon
mpcobj = mpc(plant, Ts, p, m);
mpcobj.Weights.OutputVariables = 1;
mpcobj.Weights.ManipulatedVariables = 0.1;
mpcobj.MV.Min = -0.5;
mpcobj.MV.Max =  0.5;

% Initial conditions
x = [0.5; 0.0];
y = C*x;
xref = 0;  % reference position

% Internal state object holds warm-start info
xmpc = mpcstate(mpcobj);

for k = 1:200
    % Compute control move; mpcmove uses internal warm-start information
    u = mpcmove(mpcobj, xmpc, y, xref);

    % Apply to plant
    x = A*x + B*u;
    y = C*x;

    % In Simulink, the MPC Controller block plays the same role
end
