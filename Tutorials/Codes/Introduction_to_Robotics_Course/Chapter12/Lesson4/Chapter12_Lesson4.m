% ROS1 MATLAB TF broadcasting (usage)
rosshutdown;
rosinit;

tftree = rostf;  % TF interface

parent = 'base_link';
child  = 'lidar_frame';

trvec = [0.25 0 0.12];   % translation
rpy   = [0 0 0];         % roll-pitch-yaw
quat  = eul2quat(rpy);

msg = rosmessage('geometry_msgs/TransformStamped');
msg.Header.FrameId = parent;
msg.ChildFrameId   = child;
msg.Transform.Translation.X = trvec(1);
msg.Transform.Translation.Y = trvec(2);
msg.Transform.Translation.Z = trvec(3);
msg.Transform.Rotation.W = quat(1);
msg.Transform.Rotation.X = quat(2);
msg.Transform.Rotation.Y = quat(3);
msg.Transform.Rotation.Z = quat(4);

rate = robotics.Rate(30);
while true
    msg.Header.Stamp = rostime('now');
    sendTransform(tftree, msg);
    waitfor(rate);
end
      
