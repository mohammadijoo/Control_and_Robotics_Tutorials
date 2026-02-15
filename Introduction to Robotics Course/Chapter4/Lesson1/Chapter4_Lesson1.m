
% Mobility counting for a spatial mechanism
N = 7;                 % links including base
f = ones(1,6);         % 6 revolute joints
J = numel(f);
M = 6*(N-1-J) + sum(f);
disp("Mobility = " + M);

% MATLAB Robotics System Toolbox provides a 'rigidBodyTree'
robot = rigidBodyTree('DataFormat','row');
for i = 1:6
    body = rigidBody("link" + i);
    joint = rigidBodyJoint("joint" + i,'revolute');
    setFixedTransform(joint, eye(4));  % placeholder transform
    body.Joint = joint;
    if i==1
        addBody(robot, body, robot.BaseName);
    else
        addBody(robot, body, "link" + (i-1));
    end
end
showdetails(robot);
      