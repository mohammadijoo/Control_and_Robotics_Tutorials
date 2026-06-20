% Assume ROS master is already running and a robot listens on /cmd_vel
rosinit;  % connect to ROS network (only once)

subSpeech  = rossubscriber("/speech_velocity", "std_msgs/Float64");
subGesture = rossubscriber("/gesture_velocity", "std_msgs/Float64");
pubCmd     = rospublisher("/cmd_vel", "geometry_msgs/Twist");

% Variance estimates for the two channels (speech, gesture)
sigma_s = 0.15;
sigma_g = 0.05;

w_s = (1/sigma_s^2) / (1/sigma_s^2 + 1/sigma_g^2);
w_g = (1/sigma_g^2) / (1/sigma_s^2 + 1/sigma_g^2);

rate = rosrate(50);  % 50 Hz fusion loop

while true
    speechMsg  = receive(subSpeech, 0.1);
    gestureMsg = receive(subGesture, 0.1);

    v_s = speechMsg.Data;
    v_g = gestureMsg.Data;

    % fused velocity command: v = w_s * v_s + w_g * v_g
    v_fused = w_s * v_s + w_g * v_g;

    twistMsg = rosmessage(pubCmd);
    twistMsg.Linear.X  = v_fused;
    twistMsg.Angular.Z = 0.0;
    send(pubCmd, twistMsg);

    waitfor(rate);
end

rosshutdown;
      
