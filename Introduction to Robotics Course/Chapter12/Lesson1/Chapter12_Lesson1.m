% matlab_subscribe_plot.m
% Requires ROS Toolbox
node = ros2node("/matlab_node");
sub  = ros2subscriber(node, "/sine", "std_msgs/Float64");

data = zeros(1,200);
for k = 1:200
    msg = receive(sub, 1.0);   % wait up to 1 second
    data(k) = msg.data;
end

plot(data); grid on;
title("Received sine samples");
xlabel("k"); ylabel("sine(k)");
      
