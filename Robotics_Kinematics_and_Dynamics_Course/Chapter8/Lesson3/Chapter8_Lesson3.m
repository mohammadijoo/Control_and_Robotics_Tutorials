function [J, kappa] = jacobian2R_cond(q1, q2, l1, l2)
%JACOBIAN2R_COND Jacobian and condition number for planar 2R arm

s1  = sin(q1);
c1  = cos(q1);
s12 = sin(q1 + q2);
c12 = cos(q1 + q2);

J = [ -l1 * s1 - l2 * s12, -l2 * s12;
       l1 * c1 + l2 * c12,  l2 * c12 ];

if rank(J) < 2
    kappa = Inf;
else
    % 2-norm condition number
    kappa = cond(J, 2);
end
end
      
