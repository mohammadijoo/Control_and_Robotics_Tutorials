% Define rigid body tree for 2R planar robot (z-axes, xy-plane)
robot = robotics.RigidBodyTree("DataFormat","row");

L1 = 1.0; L2 = 0.8;

body1 = robotics.RigidBody("link1");
jnt1  = robotics.Joint("joint1","revolute");
setFixedTransform(jnt1, trvec2tform([0 0 0]));
jnt1.JointAxis = [0 0 1];
body1.Joint = jnt1;
addBody(robot, body1, robot.BaseName);

body2 = robotics.RigidBody("link2");
jnt2  = robotics.Joint("joint2","revolute");
% link1 to link2 translation along x by L1
setFixedTransform(jnt2, trvec2tform([L1 0 0]));
jnt2.JointAxis = [0 0 1];
body2.Joint = jnt2;
addBody(robot, body2, "link1");

% Add end-effector frame at distance L2 along x from link2
ee = robotics.RigidBody("tool");
setFixedTransform(ee.Joint, trvec2tform([L2 0 0]));
addBody(robot, ee, "link2");

q = [0.5 -0.3];
J = geometricJacobian(robot, q, "tool");
disp(J);  % 6 x 2 Jacobian
      
