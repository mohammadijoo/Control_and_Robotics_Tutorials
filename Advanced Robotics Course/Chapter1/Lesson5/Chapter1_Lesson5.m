% Define a simple 2R planar robot in rigidBodyTree
robot = rigidBodyTree('DataFormat','row','MaxNumBodies',3);
L1 = 1.0; L2 = 0.7;

body1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1','revolute');
setFixedTransform(joint1,trvec2tform([0 0 0]));
joint1.JointAxis = [0 0 1];
body1.Joint = joint1;
addBody(robot,body1,'base');

body2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint2,trvec2tform([L1 0 0]));
joint2.JointAxis = [0 0 1];
body2.Joint = joint2;
addBody(robot,body2,'link1');

ee = rigidBody('tool');
setFixedTransform(ee.Joint,trvec2tform([L2 0 0]));
addBody(robot,ee,'link2');

% Workspace obstacle as a box
env = {collisionBox(0.4, 0.2, 0.5)}; % [X Y Z] dimensions
Tobs = trvec2tform([0.6 0.0 0.25]);  % pose in workspace
env{1}.Pose = Tobs;

% Joint grid approximation
N = 100;
theta1_vals = linspace(-pi, pi, N);
theta2_vals = linspace(-pi, pi, N);
occ = false(N, N);

for i = 1:N
    for j = 1:N
        q = [theta1_vals(i) theta2_vals(j)];
        isColliding = checkCollision(robot, q, env, ...
                                     'IgnoreSelfCollision','on');
        occ(i,j) = isColliding;
    end
end

% At this point, occ(i,j) is a discrete approximation of C_obs
% that can be exported or used in planners.
      
