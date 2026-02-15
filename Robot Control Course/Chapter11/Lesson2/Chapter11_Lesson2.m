
% Assume 'robot' is a robotics.RigidBodyTree with gravity etc. configured.
% linearize about a given configuration q0, qdot0, tau0:
q0     = [0; 0];       % 2-DOF example
qdot0  = [0; 0];
tau0   = [0; 0];

x0     = [q0; qdot0];  % operating point
u0     = tau0;

% Use a custom function or robotics toolbox helper to compute A,B,C,D
% e.g., A,B,C,D = linearizeRobot(robot, x0, u0);
% For illustration, assume we already have Ac,Bc,Cc,Dc:

% Discretize
Ts = 1e-3;
sysc = ss(Ac, Bc, Cc, Dc);
sysd = c2d(sysc, Ts, 'zoh');
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;

% Check observability
Ob = obsv(Ad, Cd);
if rank(Ob) < size(Ad,1)
    error('System not observable at this operating point.');
end

% Choose observer poles (discrete-time) faster than controller poles
observer_poles = [0.3 0.35 0.4 0.45];  % example
Ld = place(Ad', Cd', observer_poles).'; % duality

% In Simulink:
% - Use a State-Space block for the plant (Ad,Bd,Cd,Dd).
% - Add a second State-Space block for the observer with input [u; y]
%   and internal dynamics: xhat(k+1) = Ad*xhat + Bd*u + Ld*(y - C*xhat).
% - Feed xhat to your state-feedback controller instead of x.
