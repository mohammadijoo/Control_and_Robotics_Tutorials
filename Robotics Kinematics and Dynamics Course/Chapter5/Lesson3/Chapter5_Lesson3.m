function T = dh_transform(theta, d, a, alpha)
% Single DH transform (standard convention)
ct = cos(theta);
st = sin(theta);
ca = cos(alpha);
sa = sin(alpha);

T = [ ct, -st * ca,  st * sa, a * ct;
      st,  ct * ca, -ct * sa, a * st;
      0,        sa,       ca,    d;
      0,         0,        0,    1 ];
end

function T = fk_dh(q, a, alpha, d, theta_offset)
% Generic DH FK
if nargin < 5
    theta_offset = zeros(size(q));
end
n = numel(q);
T = eye(4);
for i = 1:n
    theta_i = q(i) + theta_offset(i);
    A_i = dh_transform(theta_i, d(i), a(i), alpha(i));
    T = T * A_i;
end
end

function T02 = fk_planar2R(q1, q2, l1, l2)
% Planar 2R example
q = [q1; q2];
a = [l1; l2];
alpha = [0; 0];
d = [0; 0];
theta_offset = [0; 0];
T02 = fk_dh(q, a, alpha, d, theta_offset);
end
      
