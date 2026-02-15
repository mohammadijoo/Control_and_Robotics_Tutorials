function [x_next, r, done] = robot_mdp_step(x, u, dt, Q, R)
% robot_mdp_step  One transition of a robot MDP.
%   x     : state vector [q; dq]
%   u     : control vector
%   dt    : time step
%   Q, R  : cost weighting matrices

n = numel(x) / 2;
q = x(1:n);
dq = x(n+1:end);

% Joint and action limits
q_limit = pi * ones(n, 1);
u_limit = 10 * ones(size(u));

u = max(-u_limit, min(u_limit, u));

% Simple double-integrator dynamics
ddq = u;  % placeholder for full rigid-body dynamics

dq_next = dq + dt * ddq;
q_next = q + dt * dq_next;

q_next = max(-q_limit, min(q_limit, q_next));

x_next = [q_next; dq_next];

% Quadratic cost and reward
dx = x_next;  % track zero state
cost = dx' * Q * dx + u' * R * u;
r = -cost;

done = norm(x_next) > 1e3;
end
      
