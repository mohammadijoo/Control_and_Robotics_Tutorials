% Define 2R planar manipulator model
robot = rigidBodyTree("DataFormat","column","MaxNumBodies",3);

L1 = 1.0; L2 = 0.7;
m1 = 2.0; m2 = 1.0;

body1 = rigidBody("link1");
jnt1  = rigidBodyJoint("joint1","revolute");
setFixedTransform(jnt1,trvec2tform([0 0 0]));
jnt1.JointAxis = [0 0 1];
body1.Joint = jnt1;
addBody(robot,body1,"base");

body2 = rigidBody("link2");
jnt2  = rigidBodyJoint("joint2","revolute");
setFixedTransform(jnt2,trvec2tform([L1 0 0]));
jnt2.JointAxis = [0 0 1];
body2.Joint = jnt2;
addBody(robot,body2,"link1");

ee = rigidBody("ee");
setFixedTransform(ee.Joint,trvec2tform([L2 0 0]));
addBody(robot,ee,"link2");

robot.Gravity = [0 -9.81 0];

q = deg2rad([40; 30]);

% End-effector external wrench (Fx, Fy, Fz, Mx, My, Mz)
wrenchExt = [0; -20; 0; 0; 0; 0];

% Spatial Jacobian at end-effector
J = geometricJacobian(robot,q,"ee");   % 6 x n

% Gravity torques (special case of inverse dynamics)
tau_g = inverseDynamics(robot,q,zeros(2,1),zeros(2,1));

% Contact constraint: x(q) = x0 at end-effector (vertical wall)
T_ee = getTransform(robot,q,"ee");
x = T_ee(1,4);     % horizontal position
x0 = x;            % choose current position as constrained one
% Compute derivative of phi(q) = x(q) - x0 with respect to q
Jphi = J(1,:);     % first row corresponds to Fx direction

lambda = 50.0;     % desired normal reaction at wall

tau_contact = Jphi.' * lambda;
tau_ext = J.' * wrenchExt;

tau_a = tau_g + tau_ext + tau_contact;

disp("Static equilibrium torques with constraint:");
disp(tau_a);

% Simulink note:
% A Simulink model can use a Rigid Body Tree block (Simscape Multibody)
% and Joint blocks with zero velocity and acceleration inputs.
% Feeding the same configuration q and external wrench into a
% "Joint Torques" output gives the same tau_a in steady state.
      
