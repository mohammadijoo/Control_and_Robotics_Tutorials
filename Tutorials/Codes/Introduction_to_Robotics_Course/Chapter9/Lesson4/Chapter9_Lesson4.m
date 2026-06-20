% Rotation about z by 90 deg and translation [1 2 0]
R_AB = axang2rotm([0 0 1 pi/2]);
p_AB = [1;2;0];

T_AB = eye(4);
T_AB(1:3,1:3) = R_AB;
T_AB(1:3,4) = p_AB;

x_B = [1;0;0;1];
x_A = T_AB * x_B;

disp("x_A = ");
disp(x_A(1:3));

% Inverse
T_BA = eye(4);
T_BA(1:3,1:3) = R_AB';
T_BA(1:3,4)   = -R_AB' * p_AB;
disp("T_AB*T_BA = ");
disp(T_AB*T_BA);
