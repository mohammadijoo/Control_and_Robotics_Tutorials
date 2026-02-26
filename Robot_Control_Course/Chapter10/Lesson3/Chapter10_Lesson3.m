
Ts = 0.01;
J  = 0.05;

A = [1 Ts;
     0 1];
B = [0;
     Ts/J];
C = eye(2);     % full state output
D = [0; 0];

plant = ss(A, B, C, D, Ts);

% Horizon and weights
p = 20;   % prediction horizon
m = 3;    % control horizon

mpcobj = mpc(plant, Ts, p, m);
mpcobj.Weights.OutputVariables = [50 1];
mpcobj.Weights.ManipulatedVariables = 0.1;

% Joint and torque constraints
q_min = -1.5; q_max = 1.5;
dq_max = 2.0;
tau_max = 2.0;

mpcobj.MV.Min = -tau_max;
mpcobj.MV.Max =  tau_max;

% Output constraints (q, dq)
mpcobj.OV(1).Min = q_min;
mpcobj.OV(1).Max = q_max;
mpcobj.OV(2).Min = -dq_max;
mpcobj.OV(2).Max =  dq_max;

% Reference (desired joint angle)
yref = [1; 0];

x = [0; 0];
u = 0;

for k = 1:50
    % Compute constrained MPC move
    u = mpcmove(mpcobj, x, x, yref);
    % Simulate plant
    x = A*x + B*u;
    fprintf("step %d: q = %.3f, dq = %.3f, tau = %.3f\n", k, x(1), x(2), u);
end
