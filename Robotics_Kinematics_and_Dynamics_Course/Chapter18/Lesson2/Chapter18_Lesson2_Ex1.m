% Define 2R planar arm as a rigidBodyTree
robot = rigidBodyTree('DataFormat','row');

body1 = rigidBody('link1');
jnt1  = rigidBodyJoint('j1','revolute');
setFixedTransform(jnt1, trvec2tform([0 0 0]));
jnt1.JointAxis = [0 0 1];
body1.Joint = jnt1;
addBody(robot, body1, 'base');

body2 = rigidBody('link2');
jnt2  = rigidBodyJoint('j2','revolute');
setFixedTransform(jnt2, trvec2tform([l1 0 0]));
jnt2.JointAxis = [0 0 1];
body2.Joint = jnt2;
addBody(robot, body2, 'link1');

tool = rigidBody('tool');
setFixedTransform(tool.Joint, trvec2tform([l2 0 0]));
addBody(robot, tool, 'link2');

q = [q1, q2]; % row vector
[J6, ~] = geometricJacobian(robot, q, 'tool'); % 6 x 2 Jacobian
      
