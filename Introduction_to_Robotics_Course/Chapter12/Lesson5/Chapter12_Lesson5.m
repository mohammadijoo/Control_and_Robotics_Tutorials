% controller_node.m
node = ros2node("/matlab_controller");
sub  = ros2subscriber(node, "/sensor", "std_msgs/Float64");
pub  = ros2publisher(node, "/cmd", "std_msgs/Float64");

Kp = 1.5; r = 1.0;

rate = ros2rate(node, 50);  % 50 Hz loop
while true
    msg = receive(sub, 0.1);      % timeout 0.1 s
    y = msg.data;
    u = Kp * (r - y);

    out = ros2message(pub);
    out.data = u;
    send(pub, out);

    waitfor(rate);
end
