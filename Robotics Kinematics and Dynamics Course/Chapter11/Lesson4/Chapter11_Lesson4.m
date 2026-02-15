function qddot = two_link_forward_dynamics(t, state, tau, p)
% state = [q1; q2; q1dot; q2dot]
q1 = state(1); q2 = state(2);
q1dot = state(3); q2dot = state(4);

l1 = p.l1; c1 = p.c1; c2 = p.c2;
m1 = p.m1; m2 = p.m2;
I1 = p.I1; I2 = p.I2;
g = p.g;

cos2 = cos(q2);
sin2 = sin(q2);

M11 = I1 + I2 + m1 * c1^2 + m2 * (l1^2 + c2^2 + 2 * l1 * c2 * cos2);
M12 = I2 + m2 * (c2^2 + l1 * c2 * cos2);
M22 = I2 + m2 * c2^2;
M = [M11, M12; M12, M22];

h = m2 * l1 * c2 * sin2;
C = [-h * q2dot,          -h * (q1dot + q2dot);
      h * q1dot,           0];

g1 = (m1 * c1 + m2 * l1) * g * cos(q1) + m2 * c2 * g * cos(q1 + q2);
g2 = m2 * c2 * g * cos(q1 + q2);
gvec = [g1; g2];

rhs = tau - C * [q1dot; q2dot] - gvec;
qdd = M \ rhs;

qddot = qdd;
end
      
