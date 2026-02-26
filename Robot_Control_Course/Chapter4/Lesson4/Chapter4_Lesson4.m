
function dq = two_task_hierarchy(J1, dx1, J2, dx2, lambda1, lambda2)
% TWO_TASK_HIERARCHY  Two-level kinematic task hierarchy.
%   J1, dx1 : primary task Jacobian and desired velocity
%   J2, dx2 : secondary task Jacobian and desired velocity
%   lambda1, lambda2 : damping factors for pseudoinverse

if nargin < 5
    lambda1 = 0.0;
    lambda2 = 0.0;
end

[m1, n] = size(J1);
I = eye(n);

if lambda1 > 0
    J1_pinv = J1' / (J1 * J1' + (lambda1^2) * eye(m1));
else
    J1_pinv = pinv(J1);
end

dq1 = J1_pinv * dx1(:);
N1 = I - J1_pinv * J1;

J2_bar = J2 * N1;
[m2, ~] = size(J2_bar);
if lambda2 > 0
    J2_bar_pinv = J2_bar' / (J2_bar * J2_bar' + (lambda2^2) * eye(m2));
else
    J2_bar_pinv = pinv(J2_bar);
end

dx2_tilde = dx2(:) - J2 * dq1;
dq2_null = J2_bar_pinv * dx2_tilde;

dq = dq1 + N1 * dq2_null;
