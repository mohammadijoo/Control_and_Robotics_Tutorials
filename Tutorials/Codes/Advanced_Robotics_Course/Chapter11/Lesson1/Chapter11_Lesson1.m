% Example: log a kinesthetic demonstration from ROS joint states
rosshutdown;
rosinit;  % connect to ROS master

jointSub = rossubscriber('/joint_states', 'sensor_msgs/JointState');

t0 = rostime('now');
times = [];
Q = [];

for k = 1:1000
    msg = receive(jointSub);
    t = double(msg.Header.Stamp.Sec - t0.Sec) + ...
        double(msg.Header.Stamp.Nsec - t0.Nsec) * 1e-9;
    q = msg.Position(:)';  % row vector
    times(end+1,1) = t; %#ok<AGROW>
    Q(end+1,:) = q;      %#ok<AGROW>
end

% Resample onto uniform grid for Simulink
dt = 0.01;
tGrid = (times(1):dt:times(end))';
Qres  = interp1(times, Q, tGrid, 'linear');

% Create timeseries object for Simulink
q_ts = timeseries(Qres, tGrid);

% Now q_ts can be fed into a Simulink "From Workspace" block as demonstration input.
      
