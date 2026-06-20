% Define a rigidBodyTree with a floating base
robot = rigidBodyTree("DataFormat","row","MaxNumBodies",20);

% Example: free-floating base as 6-DoF joint (in practice, use addBody etc.)
% Here we assume 'robot' has already been built from a URDF with a floating base.

n = robot.NumBodies;         % number of rigid bodies (not DoFs)
ndof = robot.NumBodies + 6;  % generalized DoFs approx.

% Random configuration and velocities
q = homeConfiguration(robot);
q = [zeros(1,6), [q.JointPosition]];   % prepend floating base coords
v = zeros(1, ndof); v(1,1:6) = [0,0,0.5,0.1,0,0];

% Mass matrix H(q)
H = massMatrix(robot, q);

% Generalized momentum p = H v'
p = (H * v.').';

disp("Generalized momentum p:");
disp(p);
      
