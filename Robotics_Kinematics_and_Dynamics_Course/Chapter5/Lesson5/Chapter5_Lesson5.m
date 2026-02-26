function T = fk_poe(S_list, M, q)
%FK_POE Forward kinematics using the product of exponentials.
%   S_list: 6 x n matrix of screw axes (space frame)
%   M     : 4 x 4 home configuration
%   q     : n x 1 vector of joint variables

n = size(S_list, 2);
T = eye(4);
for i = 1:n
    xi = S_list(:, i);
    T = T * exp_twist(xi, q(i));
end
T = T * M;
end

function T = exp_twist(xi, q)
omega = xi(1:3);
v = xi(4:6);
theta = q;

if norm(omega) < 1e-9
    R = eye(3);
    p = v * theta;
else
    omega = omega / norm(omega);
    wx = omega(1); wy = omega(2); wz = omega(3);
    W = [   0, -wz,  wy;
          wz,   0, -wx;
         -wy,  wx,   0 ];
    W2 = W * W;
    R = eye(3) + sin(theta) * W + (1 - cos(theta)) * W2;
    V = eye(3) * theta + (1 - cos(theta)) * W + (theta - sin(theta)) * W2;
    p = V * v;
end

T = eye(4);
T(1:3,1:3) = R;
T(1:3,4) = p;
end
      
