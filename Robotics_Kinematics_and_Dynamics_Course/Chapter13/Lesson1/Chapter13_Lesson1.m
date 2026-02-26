% Example: create a rigid body with given inertial parameters
body = rigidBody('link1');

% Mass (kg)
body.Mass = 2.0;

% Center of mass in body frame {B} (m)
body.CenterOfMass = [0.1, 0.0, 0.0];

% Inertia vector [Ixx Iyy Izz Iyz Ixz Ixy] about body frame origin (kg m^2)
body.Inertia = [0.02, 0.03, 0.04, 0.0, 0.0, 0.0];

% Add to a rigid body tree (robot model)
robot = rigidBodyTree;
addBody(robot, body, robot.BaseName);
      
