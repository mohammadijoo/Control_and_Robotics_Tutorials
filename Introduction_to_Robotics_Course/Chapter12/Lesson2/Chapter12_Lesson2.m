% Create ROS2 node
node = ros2node("/matlab_node");

% Publisher
pub = ros2publisher(node, "/u_cmd", "std_msgs/Float64");
msg = ros2message(pub);

% Subscriber with callback
sub = ros2subscriber(node, "/u_cmd", "std_msgs/Float64", ...
    @(m) disp(["u_cmd = ", num2str(m.data)]));

% Publish at 100 Hz for 1 second
rate = rateControl(100);
for k = 1:100
    msg.data = 1.0;
    send(pub, msg);
    waitfor(rate);
end
      
