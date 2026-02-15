function cost = avoidanceCost2R(q, l1, l2, qmin, qmax, eps, eps_ws)
% q: [q1; q2]
if nargin < 2, l1 = 1.0; end
if nargin < 3, l2 = 1.0; end
if nargin < 4, qmin = [-pi; -pi]; end
if nargin < 5, qmax = [ pi;  pi]; end
if nargin < 6, eps  = 1e-3; end
if nargin < 7, eps_ws = 1e-3; end

q1 = q(1); q2 = q(2);

% Jacobian
s1  = sin(q1);  c1  = cos(q1);
s12 = sin(q1+q2); c12 = cos(q1+q2);
J = [ -l1*s1 - l2*s12, -l2*s12;
       l1*c1 + l2*c12,  l2*c12 ];

% Manipulability
JJt = J*J.';
w = sqrt(det(JJt));
phi_sing = 1/(w^2 + eps);

% Joint-limit penalty
phi_joint = 0;
for i = 1:2
    qi = q(i); ql = qmin(i); qu = qmax(i);
    phi_joint = phi_joint + 1/(qi - ql)^2 + 1/(qu - qi)^2;
end

% Workspace penalty
x = l1*cos(q1) + l2*cos(q1+q2);
y = l1*sin(q1) + l2*sin(q1+q2);
r = hypot(x, y);
rmin = abs(l1 - l2);
rmax = l1 + l2;
d_ws = min(r - rmin, rmax - r);
phi_ws = 1/(d_ws^2 + eps_ws);

cost = phi_sing + phi_joint + phi_ws;
end
      
