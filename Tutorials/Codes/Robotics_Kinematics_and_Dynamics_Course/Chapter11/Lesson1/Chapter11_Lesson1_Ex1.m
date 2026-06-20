robot = rigidBodyTree('DataFormat','column','MaxNumBodies',1);
body = rigidBody('link1');
jnt  = rigidBodyJoint('joint1','revolute');
setFixedTransform(jnt, trvec2tform([0 0 0]));
jnt.JointAxis = [0 0 1];
body.Joint = jnt;
addBody(robot, body, robot.BaseName);

% Assign mass properties
body.Mass = m;
body.CenterOfMass = [0 0 -l]; % simple example
body.Inertia = [0 0 0 0 0 0];

q    = 0.5;
qdot = 0.0;
qdd  = 0.0;
tau  = inverseDynamics(robot, q, qdot, qdd); % tau from M, C, g
      
