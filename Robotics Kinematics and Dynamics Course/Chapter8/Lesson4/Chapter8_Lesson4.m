% Define a simple 2R planar robot as a rigidBodyTree
robot = rigidBodyTree("DataFormat","row","MaxNumBodies",3);

% First body
body1 = rigidBody("link1");
jnt1 = rigidBodyJoint("joint1","revolute");
setFixedTransform(jnt1, trvec2tform([0 0 0]));
jnt1.JointAxis = [0 0 1];
body1.Joint = jnt1;
addBody(robot, body1, "base");

% Second body
body2 = rigidBody("link2");
jnt2 = rigidBodyJoint("joint2","revolute");
setFixedTransform(jnt2, trvec2tform([1.0 0 0])); % link1 length
jnt2.JointAxis = [0 0 1];
body2.Joint = jnt2;
addBody(robot, body2, "link1");

% End-effector frame
tool = rigidBody("tool");
setFixedTransform(tool.Joint, trvec2tform([0.7 0 0])); % link2 length
addBody(robot, tool, "link2");

% Joint limits
robot.Bodies{1}.Joint.PositionLimits = [-pi pi];
robot.Bodies{2}.Joint.PositionLimits = [-pi pi];

% Sampling
n1 = 200;
n2 = 200;
th1_vals = linspace(-pi, pi, n1);
th2_vals = linspace(-pi, pi, n2);

X = zeros(n1 * n2, 1);
Y = zeros(n1 * n2, 1);

k = 1;
for i = 1:n1
    for j = 1:n2
        config = [th1_vals(i) th2_vals(j)];
        T = getTransform(robot, config, "tool", "base");
        X(k) = T(1,4);
        Y(k) = T(2,4);
        k = k + 1;
    end
end

figure;
scatter(X, Y, 5, ".");

axis equal;
xlabel("x [m]");
ylabel("y [m]");
title("2R planar workspace (MATLAB rigidBodyTree)");
      
