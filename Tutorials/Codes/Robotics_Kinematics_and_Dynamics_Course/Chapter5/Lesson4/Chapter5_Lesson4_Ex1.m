robot = rigidBodyTree("DataFormat","row","MaxNumBodies",8);

% Define 7 revolute joints with simple kinematics (toy example)
prevBodyName = robot.BaseName;
for i = 1:7
    body = rigidBody("body" + string(i));
    joint = rigidBodyJoint("joint" + string(i), "revolute");
    % Set joint transform using DH-like parameters
    setFixedTransform(joint, trvec2tform([0.2, 0, 0]));
    joint.JointAxis = [0 0 1]; % all rotate about z-axis here
    body.Joint = joint;
    addBody(robot, body, prevBodyName);
    prevBodyName = body.Name;
end

% End-effector offset
tool = rigidBody("tool");
tform_tool = trvec2tform([0, 0, 0.1]);
setFixedTransform(tool.Joint, tform_tool);
addBody(robot, tool, prevBodyName);

% Example configuration
q = [0 0.1 -0.2 0.3 -0.1 0.2 0.05];
T = getTransform(robot, q, "tool", robot.BaseName)

% In Simulink, the 'Rigid Body Tree' block can import 'robot' and compute FK
      
