function qdot = planar3r_min_norm_qdot(q, L, xdot_des)
% q: 3x1, L: 3x1, xdot_des: 2x1

q1 = q(1); q2 = q(2); q3 = q(3);
l1 = L(1); l2 = L(2); l3 = L(3);

s1 = sin(q1); c1 = cos(q1);
s12 = sin(q1 + q2); c12 = cos(q1 + q2);
s123 = sin(q1 + q2 + q3); c123 = cos(q1 + q2 + q3);

dx_dq1 = -l1 * s1 - l2 * s12 - l3 * s123;
dx_dq2 = -l2 * s12 - l3 * s123;
dx_dq3 = -l3 * s123;

dy_dq1 =  l1 * c1 + l2 * c12 + l3 * c123;
dy_dq2 =  l2 * c12 + l3 * c123;
dy_dq3 =  l3 * c123;

J = [dx_dq1, dx_dq2, dx_dq3;
     dy_dq1, dy_dq2, dy_dq3];

% Minimum-norm solution via pseudoinverse
qdot = pinv(J) * xdot_des;
end
      
