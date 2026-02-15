function Vlinks = forwardVelocityChain(Tlist, Slist, qdot, V0)
% FORWARDVELOCITYCHAIN Propagate twists along a serial chain.
%   Tlist : 4x4xn array of relative transforms T_{i-1,i}(q_i)
%   Slist : 6xn matrix, column i is screw axis S_i in frame i
%   qdot  : nx1 vector of joint velocities
%   V0    : 6x1 base twist (optional, default zero)
%
%   Vlinks: 6xn matrix, column i is twist of link i in frame i

if nargin < 4
    Vprev = zeros(6,1);
else
    Vprev = V0(:);
end

n = size(Slist, 2);
Vlinks = zeros(6, n);

for i = 1:n
    Ti = Tlist(:,:,i);
    Ad_inv = adjoint(inv(Ti));
    Vprev_in_i = Ad_inv * Vprev;
    Si = Slist(:,i);
    Vi = Vprev_in_i + Si * qdot(i);
    Vlinks(:,i) = Vi;
    Vprev = Vi;
end
end

function Ad = adjoint(T)
R = T(1:3,1:3);
p = T(1:3,4);
p_hat = skew(p);
Ad = [R, zeros(3,3); p_hat*R, R];
end

function W = skew(w)
wx = w(1); wy = w(2); wz = w(3);
W = [   0, -wz,  wy;
      wz,   0, -wx;
     -wy,  wx,   0];
end
      
