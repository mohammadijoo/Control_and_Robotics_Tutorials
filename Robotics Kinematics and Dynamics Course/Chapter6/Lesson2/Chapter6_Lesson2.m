function [theta1_solutions, theta2_solutions] = ik2R(x, y, l1, l2)
%IK2R Analytical IK for a planar 2R arm.
%   Returns two-column vectors of theta1 and theta2 (in radians).

r2 = x.^2 + y.^2;
theta1_solutions = [];
theta2_solutions = [];

if r2 > (l1 + l2)^2 || r2 < (l1 - l2)^2
    return; % no real solution
end

c2 = (r2 - l1^2 - l2^2) / (2*l1*l2);
c2 = max(-1,min(1,c2));
s2_abs = sqrt(max(0,1 - c2^2));

phi = atan2(y, x);

for s2 = [s2_abs, -s2_abs]
    theta2 = atan2(s2, c2);
    k1 = l1 + l2*c2;
    k2 = l2*s2;
    psi = atan2(k2, k1);
    theta1 = phi - psi;
    theta1_solutions(end+1,1) = theta1; %#ok<AGROW>
    theta2_solutions(end+1,1) = theta2; %#ok<AGROW>
end
      
