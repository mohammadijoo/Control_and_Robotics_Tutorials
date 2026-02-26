function [x, xdot, xddot, J, Jdot] = planar2R_kin(q, dq, ddq, l1, l2)
% q, dq, ddq are 2x1 vectors

q1  = q(1);   q2  = q(2);
dq1 = dq(1);  dq2 = dq(2);

x = [l1*cos(q1) + l2*cos(q1 + q2);
     l1*sin(q1) + l2*sin(q1 + q2)];

J = [ -l1*sin(q1) - l2*sin(q1 + q2),   -l2*sin(q1 + q2);
       l1*cos(q1) + l2*cos(q1 + q2),    l2*cos(q1 + q2) ];

Jdot = [ -dq1*l1*cos(q1) - (dq1 + dq2)*l2*cos(q1 + q2),  -(dq1 + dq2)*l2*cos(q1 + q2);
         -dq1*l1*sin(q1) - (dq1 + dq2)*l2*sin(q1 + q2),  -(dq1 + dq2)*l2*sin(q1 + q2) ];

xdot  = J * dq;
xddot = J * ddq + Jdot * dq;
end
      
