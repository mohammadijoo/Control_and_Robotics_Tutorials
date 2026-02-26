function [M, Cqdot, g] = planar2R_dynamics(q, qdot, params)
%PLANAR2R_DYNAMICS Compute M(q), C(q, qdot) qdot, g(q) for planar 2R arm.
%   q      : [q1; q2]
%   qdot   : [dq1; dq2]
%   params : struct with fields m1, m2, l1, lc1, lc2, I1, I2, g0

q1  = q(1);
q2  = q(2);
dq1 = qdot(1);
dq2 = qdot(2);

m1  = params.m1;
m2  = params.m2;
l1  = params.l1;
lc1 = params.lc1;
lc2 = params.lc2;
I1  = params.I1;
I2  = params.I2;
g0  = params.g0;

c2 = cos(q2);
s2 = sin(q2);

% Inertia matrix
M11 = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2);
M12 = I2 + m2*(lc2^2 + l1*lc2*c2);
M22 = I2 + m2*lc2^2;

M = [M11, M12;
     M12, M22];

% Coriolis/centrifugal vector
h1 = -m2*l1*lc2*s2*(2*dq1*dq2 + dq2^2);
h2 =  m2*l1*lc2*s2*dq1^2;

Cqdot = [h1; h2];

% Gravity vector
g1 = (m1*lc1 + m2*l1)*g0*cos(q1) + m2*lc2*g0*cos(q1 + q2);
g2 = m2*lc2*g0*cos(q1 + q2);
g  = [g1; g2];
end
      
