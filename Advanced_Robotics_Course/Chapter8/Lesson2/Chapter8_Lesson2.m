function flag = isAntipodalGrasp(p1, n1, p2, n2, mu, tol)
%ISANTIPODALGRASP Sufficient antipodal condition in 3D.
%   p1, p2 : 3x1 positions
%   n1, n2 : 3x1 outward normals
%   mu     : friction coefficient
%   tol    : angular tolerance (radians)

if nargin < 6
    tol = 1e-3;
end

p1 = p1(:); p2 = p2(:);
n1 = n1(:); n2 = n2(:);

n1 = n1 / norm(n1);
n2 = n2 / norm(n2);

d = (p2 - p1);
d = d / norm(d);

phi = atan(mu);

theta1 = angleBetween(-n1, d);
theta2 = angleBetween(n2, -d);
thetaN = angleBetween(n1, -n2);

flag = (theta1 <= phi + tol) && ...
       (theta2 <= phi + tol) && ...
       (thetaN >= pi - 0.3);
end

function th = angleBetween(a, b)
au = a / norm(a);
bu = b / norm(b);
c = max(-1.0, min(1.0, au' * bu));
th = acos(c);
end
      
