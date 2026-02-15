% Rotation about z-axis by 45 degrees
theta = pi/4;
R = [cos(theta) -sin(theta) 0;
     sin(theta)  cos(theta) 0;
     0           0          1];

% Translation vector
p = [0.5; 0.2; 0.0];

% Build homogeneous transform manually
T = [R, p;
     0 0 0 1];

% Use Robotics System Toolbox utilities
T_rot = axang2tform([0 0 1 theta]);
T_tr  = trvec2tform([0.5 0.2 0.0]);
T_toolbox = T_tr * T_rot;

% Transform a point
x_B = [0.1; 0.0; 0.0; 1.0];   % homogeneous coordinates
x_A = T * x_B;                % coordinates in frame A
      
