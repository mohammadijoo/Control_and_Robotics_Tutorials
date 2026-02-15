
function qdot = twoTaskWBC(q, J1, xdot1_star, J2, xdot2_star, lambda)
%TWOTASKWBC Hierarchical two-task velocity control.
%
% q           : current joint configuration (unused here, but typically used
%               to compute J1, J2 from a rigidBodyTree model)
% J1          : Jacobian of high-priority task (m1 x n)
% xdot1_star  : desired task-1 velocity (m1 x 1)
% J2          : Jacobian of second-priority task (m2 x n)
% xdot2_star  : desired task-2 velocity (m2 x 1)
% lambda      : damping coefficient

if nargin < 6
    lambda = 1e-3;
end

J1_pinv = damped_pinv(J1, lambda);
qdot1 = J1_pinv * xdot1_star;

N1 = eye(size(J1, 2)) - J1_pinv * J1;

J2_eff = J2 * N1;
J2_eff_pinv = damped_pinv(J2_eff, lambda);

residual2 = xdot2_star - J2 * qdot1;
qdot2_corr = N1 * (J2_eff_pinv * residual2);

qdot = qdot1 + qdot2_corr;
end

function J_pinv = damped_pinv(J, lambda)
[m, n] = size(J);
if m >= n
    A = J' * J + (lambda^2) * eye(n);
    J_pinv = A \ J';
else
    A = J * J' + (lambda^2) * eye(m);
    J_pinv = J' / A;
end
end
