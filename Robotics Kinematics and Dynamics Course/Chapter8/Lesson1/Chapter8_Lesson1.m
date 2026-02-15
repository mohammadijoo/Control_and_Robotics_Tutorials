% Define 2R rigid body tree (conceptual; details omitted)
robot = robotics.RigidBodyTree("DataFormat","row");
% ... add bodies and joints to 'robot' ...

% Example configuration [theta1 theta2]
q = [0 0];  % fully stretched

% Compute body Jacobian for end-effector
J = geometricJacobian(robot, q, robot.BodyNames{end});

% Kinematic singularity test via determinant (2D task)
detJ = det(J(1:2, 1:2));   % assume planar model
isSingular = abs(detJ) < 1e-6

% In Simulink, this logic can be embedded in a MATLAB Function block that
% takes q(t) as input and outputs a boolean singularity flag.
      
