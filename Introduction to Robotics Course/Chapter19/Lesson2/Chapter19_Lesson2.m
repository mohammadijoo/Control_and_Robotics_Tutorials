% Initialize ROS connection (ROS master already running)
rosinit;

% Subscribe to a scalar topic, e.g., "/joint1_position"
sub = rossubscriber("/joint1_position");

N = 500;
t_vec = zeros(N, 1);
q_vec = zeros(N, 1);

for k = 1:N
    msg = receive(sub);  % blocking receive
    t_vec(k) = double(msg.Header.Stamp.Sec) + ...
               1e-9 * double(msg.Header.Stamp.Nsec);
    q_vec(k) = msg.Data;
end

% Shift time origin
t_vec = t_vec - t_vec(1);

figure;
plot(t_vec, q_vec, "LineWidth", 1.5);
xlabel("time [s]");
ylabel("joint position [rad]");
grid on;

% Shut down ROS node
rosshutdown;
      
