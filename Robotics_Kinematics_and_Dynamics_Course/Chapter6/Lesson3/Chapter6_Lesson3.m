function sols = ik2R(l1, l2, x_d, y_d, tol)
%IK2R Geometric inverse kinematics for planar 2R manipulator.
%   sols is a 2xN matrix containing [q1; q2] solutions (radians).

if nargin < 5
    tol = 1e-9;
end

d_sq = x_d.^2 + y_d.^2;
d = sqrt(d_sq);

if (d > l1 + l2 + tol) || (d < abs(l1 - l2) - tol)
    sols = []; % unreachable
    return;
end

c2 = (d_sq - l1^2 - l2^2) / (2 * l1 * l2);
c2 = max(-1.0, min(1.0, c2));

disc = max(0.0, 1.0 - c2^2);
s2_candidates = [sqrt(disc), -sqrt(disc)];

phi = atan2(y_d, x_d);
sols = zeros(2, 0);

for s2 = s2_candidates
    q2 = atan2(s2, c2);
    k1 = l1 + l2 * c2;
    k2 = l2 * s2;
    psi = atan2(k2, k1);
    q1 = phi - psi;

    sols(:, end+1) = [q1; q2];
end
end
      
