% Define a 2R planar arm as a rigidBodyTree
robot = rigidBodyTree("DataFormat","row","MaxNumBodies",3);

% Link 1
body1 = rigidBody("L1");
jnt1  = rigidBodyJoint("J1","revolute");
setFixedTransform(jnt1, trvec2tform([0 0 0]));  % base to joint 1
jnt1.JointAxis = [0 0 1];                       % rotation about z
body1.Joint = jnt1;
addBody(robot, body1, robot.BaseName);

% Link 2
body2 = rigidBody("L2");
jnt2  = rigidBodyJoint("J2","revolute");
L1 = 1.0;
setFixedTransform(jnt2, trvec2tform([L1 0 0])); % along x from joint 1
jnt2.JointAxis = [0 0 1];
body2.Joint = jnt2;
addBody(robot, body2, "L1");

% End-effector frame
tool = rigidBody("tool");
setFixedTransform(tool.Joint, trvec2tform([0.7 0 0]));
addBody(robot, tool, "L2");

% Joint configuration (generalized coordinates)
q = [deg2rad(45) deg2rad(-30)];
T_ee = getTransform(robot, q, "tool", robot.BaseName);
disp(T_ee);
      
