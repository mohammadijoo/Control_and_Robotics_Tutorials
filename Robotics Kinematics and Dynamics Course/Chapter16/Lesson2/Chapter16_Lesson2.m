function [Jf, Jq, Jx] = jacobian3RPR(geom, L, pose)
% geom.B, geom.P : 3x2 matrices for base and platform points
% L   : 3x1 vector [L1; L2; L3]
% pose: 3x1 vector [x; y; phi]

x   = pose(1);
y   = pose(2);
phi = pose(3);

p = [x; y];
c = cos(phi);
s = sin(phi);
R = [c, -s; s, c];
S = [0, -1; 1, 0];

Jq = zeros(3, 3);
Jx = zeros(3, 3);

% J_q diagonal
for i = 1:3
    Jq(i, i) = -2 * L(i);
end

% J_x rows
for i = 1:3
    Bi = geom.B(i, :).';
    Pi = geom.P(i, :).';
    d = p + R * Pi - Bi;
    dix = d(1);
    diy = d(2);

    dphi = S * (R * Pi);

    dPhi_dx   = 2 * dix;
    dPhi_dy   = 2 * diy;
    dPhi_dphi = 2 * (d.' * dphi);

    Jx(i, :) = [dPhi_dx, dPhi_dy, dPhi_dphi];
end

% Forward Jacobian: J_f = - J_x^{-1} J_q
Jf = - (Jx \ Jq); % more stable than inv(Jx) * Jq
end
      
