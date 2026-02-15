function T = fkine_space(M, S_list, theta)
% FKINE_SPACE  Forward kinematics via PoE (space frame).
%   M      : 4x4 home configuration
%   S_list : 6xn screw axes, each column is S_i
%   theta  : nx1 vector of joint variables

T = eye(4);
n = size(S_list, 2);
for i = 1:n
    S = S_list(:, i);
    T = T * matrixExp6(S, theta(i));
end
T = T * M;
end

function wx = skew3(omega)
wx = [   0,       -omega(3),  omega(2);
      omega(3),      0,      -omega(1);
     -omega(2),   omega(1),     0      ];
end

function R = matrixExp3(omega, theta)
wx = skew3(omega);
R = eye(3) + sin(theta) * wx + (1 - cos(theta)) * (wx * wx);
end

function T = matrixExp6(S, theta)
omega = S(1:3);
v     = S(4:6);
normW = norm(omega);

if normW > 1e-8
    omega_unit = omega / normW;
    thetaScaled = normW * theta;
    R = matrixExp3(omega_unit, thetaScaled);
    wx = skew3(omega_unit);
    G = eye(3) * thetaScaled + (1 - cos(thetaScaled)) * wx ...
        + (thetaScaled - sin(thetaScaled)) * (wx * wx);
    p = G * (v / normW);
else
    R = eye(3);
    p = v * theta;
end

T = eye(4);
T(1:3, 1:3) = R;
T(1:3, 4)   = p;
end

% Example usage (script section):
L1 = 1.0;
L2 = 1.0;
S1 = [0;0;1; 0; 0;  0];
S2 = [0;0;1; 0;-L1; 0];
S_list = [S1, S2];
M = eye(4);
M(1,4) = L1 + L2;

theta = deg2rad([30; 45]);
T_theta = fkine_space(M, S_list, theta)
      
