function dx = closed_chain_dynamics(t, x, param)
% State x = [q; qdot]

n = numel(x) / 2;
q = x(1:n);
qdot = x(n+1:end);

[Mq, h]       = compute_dynamics(q, qdot, param);   % h = C(q,qdot) qdot + g(q)
[Jc, Jcdotqd] = compute_constraints(q, qdot, param);
Sa            = selection_matrix(param);           % n_a x n
tau           = actuator_torques(t, q, qdot, param);

m = size(Jc, 1);

K  = zeros(n + m, n + m);
rhs = zeros(n + m, 1);

K(1:n, 1:n)       = Mq;
K(1:n, n+1:end)   = -Jc.';
K(n+1:end, 1:n)   = Jc;

rhs(1:n)          = Sa.' * tau - h;
rhs(n+1:end)      = -Jcdotqd;

sol   = K \ rhs;
qddot = sol(1:n);

dx        = zeros(size(x));
dx(1:n)   = qdot;
dx(n+1:end) = qddot;
end
      
