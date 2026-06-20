% Assume 'robot' is a rigidBodyTree from Robotics System Toolbox

q   = rand(robot.NumBodies, 1);   % joint positions
qd  = rand(robot.NumBodies, 1);   % joint velocities
qdd = rand(robot.NumBodies, 1);   % joint accelerations

tau = inverseDynamics(robot, q, qd, qdd);  % uses Newton-Euler internally

% Extract gravity, Coriolis, and inertia numerically:
g_vec = inverseDynamics(robot, q, zeros(size(q)), zeros(size(q)));
Cqd   = inverseDynamics(robot, q, qd, zeros(size(q))) - g_vec;
M_col1 = inverseDynamics(robot, q, zeros(size(q)), [1; zeros(robot.NumBodies-1, 1)]) - g_vec;
      
