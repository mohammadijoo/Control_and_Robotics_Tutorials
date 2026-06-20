% Requires Robotics System Toolbox / ROS Toolbox
node = ros2node("/matlab_node");
pub  = ros2publisher(node, "/chatter", "std_msgs/String");

msg = ros2message("std_msgs/String");
for k = 1:10
    msg.data = "hello " + string(k);
    send(pub, msg);
    pause(0.5);
end

clear node pub
      
