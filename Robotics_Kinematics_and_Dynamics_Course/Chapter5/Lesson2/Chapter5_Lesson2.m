% Define a 2R planar manipulator using standard DH parameters
l1 = 1.0; l2 = 0.7;

robot = rigidBodyTree('DataFormat','row','MaxNumBodies',3);

% Link 1
body1 = rigidBody('link1');
jnt1  = rigidBodyJoint('joint1','revolute');
setFixedTransform(jnt1, [0 0 l1 0], 'dh');  % [theta d a alpha] with symbolic theta
body1.Joint = jnt1;
addBody(robot, body1, 'base');

% Link 2
body2 = rigidBody('link2');
jnt2  = rigidBodyJoint('joint2','revolute');
setFixedTransform(jnt2, [0 0 l2 0], 'dh');
body2.Joint = jnt2;
addBody(robot, body2, 'link1');

% Add end-effector frame
ee = rigidBody('tool');
setFixedTransform(ee.Joint, eye(4));
addBody(robot, ee, 'link2');

% Forward kinematics for given joint angles q
q = [0.5 -0.3];
T_0_ee = getTransform(robot, q, 'tool', 'base')
      
