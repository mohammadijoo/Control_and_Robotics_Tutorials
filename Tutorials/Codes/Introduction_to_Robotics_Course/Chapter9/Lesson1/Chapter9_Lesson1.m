% Coordinate change in MATLAB
R_BA = [0 1; -1 0];   % 90 deg CCW rotation
v_A  = [2; 1];
v_B  = R_BA * v_A;

disp("v_A ="); disp(v_A);
disp("v_B ="); disp(v_B);
disp("lengths ="); disp([norm(v_A), norm(v_B)]);

% Simulink idea:
% Use a Constant block for v_A and R_BA,
% then a Matrix Multiply block to produce v_B.
