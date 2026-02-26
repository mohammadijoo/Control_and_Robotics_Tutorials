% MATLAB ROS2 example: reproducible randomness in a loop
% (requires Robotics System Toolbox with ROS2 support)
seed = 0;
rng(seed,"twister");  % deterministic PRG

node = ros2node("/matlab_noisy_pub");
pub  = ros2publisher(node,"/noise","std_msgs/Float32");

rate = ros2rate(node,10); % 10 Hz
while true
    z = randn();          % deterministic given seed
    msg = ros2message(pub);
    msg.data = single(z);
    send(pub,msg);
    waitfor(rate);
end
      
