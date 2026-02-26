% Define a simple floating-base tree with one leg
robot = rigidBodyTree("DataFormat","column","MaxNumBodies",5);

% Floating base modeled as 6-DoF joint
base = rigidBody("base");
jb = rigidBodyJoint("base6dof","6dof");
setFixedTransform(jb, eye(4));
base.Joint = jb;
addBody(robot, base, robot.BaseName);

% Add a hip, knee, ankle chain
hip = rigidBody("hip");
j_hip = rigidBodyJoint("j_hip","revolute");
setFixedTransform(j_hip, trvec2tform([0 0 0.5]));
j_hip.JointAxis = [0 1 0];
hip.Joint = j_hip;
addBody(robot, hip, "base");

knee = rigidBody("knee");
j_knee = rigidBodyJoint("j_knee","revolute");
setFixedTransform(j_knee, trvec2tform([0 0 -0.4]));
j_knee.JointAxis = [0 1 0];
knee.Joint = j_knee;
addBody(robot, knee, "hip");

ankle = rigidBody("ankle");
j_ankle = rigidBodyJoint("j_ankle","revolute");
setFixedTransform(j_ankle, trvec2tform([0 0 -0.4]));
j_ankle.JointAxis = [1 0 0];
ankle.Joint = j_ankle;
addBody(robot, ankle, "knee");

% Configuration vector includes 6 base DoFs + 3 leg joints
q = homeConfiguration(robot);
q(1:6) = [0; 0; 0.9; 0; 0; 0]; % base position/orientation (XYZRPY)

% Forward kinematics: transform of ankle
T0ankle = getTransform(robot, q, "ankle");

% Whole-body geometric Jacobian for ankle
[J_ankle, ~] = geometricJacobian(robot, q, "ankle");
      
