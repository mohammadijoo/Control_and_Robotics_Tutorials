function T = fk_poe_matlab(S_list, M, q)
% S_list: 6xn matrix of screw axes (space frame)
% M: 4x4 home configuration
% q: nx1 vector of joint variables
n = size(S_list, 2);
T = M;
for i = n:-1:1
    S = S_list(:, i);
    se3mat = twist_hat(S);
    T = expm(se3mat * q(i)) * T;
end
end

function se3mat = twist_hat(S)
omega = S(1:3);
v = S(4:6);
se3mat = zeros(4, 4);
se3mat(1:3, 1:3) = skew3(omega);
se3mat(1:3, 4) = v;
end

function Omega = skew3(omega)
wx = omega(1); wy = omega(2); wz = omega(3);
Omega = [ 0   -wz   wy;
          wz   0   -wx;
         -wy   wx   0 ];
end
      
