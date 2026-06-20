% Axis-angle to rotation matrix, then to quaternion and Euler ZYX
omega_hat = [0; 0; 1];  % rotation about z
theta = pi/4;           % 45 degrees

R = axang2rotm([omega_hat.' theta]);   % Robotics System Toolbox
q = rotm2quat(R);                      % [w x y z] convention
eulZYX = rotm2eul(R, 'ZYX');           % [phi theta psi] in radians

% Convert back via toolbox routines
R2 = quat2rotm(q);
eul2 = rotm2eul(R2, 'ZYX');

% Numerical consistency check
disp(norm(R - R2));
disp(norm(eulZYX - eul2));

% Simulink:
% Use "Axis-Angle to Rotation Matrix" block, "Rotation Matrix to
% Quaternion" block, etc., in the Robotics System Toolbox library.
      
