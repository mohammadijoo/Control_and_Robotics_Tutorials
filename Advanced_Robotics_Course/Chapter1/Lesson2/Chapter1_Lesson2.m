% Link lengths
L1 = 1.0; L2 = 1.0;

% Build a simple planar rigidBodyTree
robot = rigidBodyTree("DataFormat","row","MaxNumBodies",2);
body1 = rigidBody("link1");
jnt1 = rigidBodyJoint("joint1","revolute");
setFixedTransform(jnt1, trvec2tform([0 0 0]));
body1.Joint = jnt1;
addBody(robot, body1, "base");

body2 = rigidBody("link2");
jnt2 = rigidBodyJoint("joint2","revolute");
setFixedTransform(jnt2, trvec2tform([L1 0 0]));
body2.Joint = jnt2;
addBody(robot, body2, "link1");

% Collision sphere as an environment object
[obsCenter, obsRadius] = deal([0.8 0.4 0.0], 0.25);
obs = collisionSphere(obsRadius);
obs.Pose = trvec2tform(obsCenter);

% Joint configuration
q = [0.3 -1.0];

% Collision check
isColliding = checkCollision(robot, q, {obs}, ...
    "IgnoreSelfCollision", true);

disp(isColliding);
      
