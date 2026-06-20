% Chapter 14 - Lesson 5: Lab: Assemble a Full Navigation Stack
%
% MATLAB (Robotics System Toolbox) example:
% - Create ROS 2 node
% - Create NavigateToPose action client
% - Send goal and wait for result
% - Subscribe to /amcl_pose for localization monitoring
%
% Prerequisites:
% - Robotics System Toolbox
% - ROS 2 environment configured (e.g., ROS_DOMAIN_ID)
% - Nav2 running and exposing /navigate_to_pose action and /amcl_pose topic

clear; clc;

node = ros2node("/chapter14_lesson5_matlab_client");

% Subscribe to AMCL pose (optional diagnostics)
amclSub = ros2subscriber(node, "/amcl_pose", "geometry_msgs/PoseWithCovarianceStamped");

% Create action client (Nav2)
ac = ros2actionclient(node, "/navigate_to_pose", "nav2_msgs/NavigateToPose");

fprintf("Waiting for action server...\n");
waitForServer(ac);

% Build goal
goalMsg = ros2message(ac);
goalMsg.pose.header.frame_id = "map";
goalMsg.pose.header.stamp = ros2time(node);

goalMsg.pose.pose.position.x = 2.0;
goalMsg.pose.pose.position.y = 0.5;
goalMsg.pose.pose.position.z = 0.0;

yaw = 0.0;
goalMsg.pose.pose.orientation.x = 0.0;
goalMsg.pose.pose.orientation.y = 0.0;
goalMsg.pose.pose.orientation.z = sin(yaw/2.0);
goalMsg.pose.pose.orientation.w = cos(yaw/2.0);

fprintf("Sending goal...\n");
gh = sendGoal(ac, goalMsg);

tStart = tic;

while ~isDone(gh)
    fb = getFeedback(gh);
    if ~isempty(fb)
        fprintf("Distance remaining: %.3f m\n", fb.distance_remaining);
    end

    if amclSub.NewMessageAvailable
        amcl = receive(amclSub, 0.01);
        x = amcl.pose.pose.position.x;
        y = amcl.pose.pose.position.y;
        fprintf("AMCL pose: (%.3f, %.3f)\n", x, y);
    end

    pause(0.1);
end

res = getResult(gh);
tGoal = toc(tStart);

fprintf("\n=== Chapter14 Lesson5 Report ===\n");
fprintf("Time-to-goal (s): %.3f\n", tGoal);
disp(res);

clear node;
