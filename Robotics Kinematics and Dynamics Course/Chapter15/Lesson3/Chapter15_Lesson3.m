function [M,h,Jc,Jcdot] = fourbar_dynamics_matrices(q, qdot, params)
% FOURBAR_DYNAMICS_MATRICES  Planar 4-bar building blocks.
%   q    : [th2; th3; th4]
%   qdot : [th2dot; th3dot; th4dot]
%   params: struct with fields a,b,c,d,m2,m3,m4,g

a = params.a; b = params.b; c = params.c; d = params.d;
m2 = params.m2; m3 = params.m3; m4 = params.m4; g = params.g;

th2 = q(1); th3 = q(2); th4 = q(3);
th2d = qdot(1); th3d = qdot(2); th4d = qdot(3);

% Inertias (simple approximation)
I2 = m2 * a^2 / 12;
I3 = m3 * b^2 / 12;
I4 = m4 * c^2 / 12;

M = diag([I2 I3 I4]);

% Gravity torques
tau2_g = -m2 * g * (a/2) * cos(th2);
tau3_g = -m3 * g * (b/2) * cos(th3);
tau4_g = -m4 * g * (c/2) * cos(th4);
h = [tau2_g; tau3_g; tau4_g];

% Constraint Jacobian
Jc = [ -a*sin(th2), -b*sin(th3),  c*sin(th4);
        a*cos(th2),  b*cos(th3), -c*cos(th4) ];

% Time derivative Jcdot
Jcdot = zeros(2,3);
Jcdot(1,1) = -a*cos(th2)*th2d;
Jcdot(1,2) = -b*cos(th3)*th3d;
Jcdot(1,3) =  c*cos(th4)*th4d;
Jcdot(2,1) = -a*sin(th2)*th2d;
Jcdot(2,2) = -b*sin(th3)*th3d;
Jcdot(2,3) = -c*sin(th4)*th4d;
      
