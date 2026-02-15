function qdot = redundancy_step(q, xdot_des, L, q_min, q_max, alpha, lambda)
% q: 3x1, xdot_des: 2x1, L: 3x1 link lengths
% q_min, q_max: 3x1 joint limits

J = planar3_jacobian(q, L);

% Damped pseudoinverse: J' * inv(J*J' + lambda^2 I)
m = size(J, 1);
J_pinv = J' * ((J*J' + (lambda^2)*eye(m)) \ eye(m));

qdot0 = J_pinv * xdot_des;
N = eye(size(J, 2)) - J_pinv * J;

% Joint-limit gradient
q_mid = 0.5 * (q_min + q_max);
span = (q_max - q_min);
normed = (q - q_mid) ./ span;
gradH = normed ./ span;

qdotH = -alpha * (N * gradH);

qdot = qdot0 + qdotH;
end

function J = planar3_jacobian(q, L)
q1 = q(1); q2 = q(2); q3 = q(3);
l1 = L(1); l2 = L(2); l3 = L(3);

s1 = sin(q1); c1 = cos(q1);
s12 = sin(q1 + q2); c12 = cos(q1 + q2);
s123 = sin(q1 + q2 + q3); c123 = cos(q1 + q2 + q3);

J = zeros(2, 3);
J(1,1) = -l1*s1 - l2*s12 - l3*s123;
J(2,1) =  l1*c1 + l2*c12 + l3*c123;
J(1,2) = -l2*s12 - l3*s123;
J(2,2) =  l2*c12 + l3*c123;
J(1,3) = -l3*s123;
J(2,3) =  l3*c123;
end
      
