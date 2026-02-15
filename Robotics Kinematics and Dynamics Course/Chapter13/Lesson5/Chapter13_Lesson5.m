% Import URDF exported from CAD
robot = importrobot("my_manipulator.urdf");

% Use joint-space data format for direct matrix computation
robot.DataFormat = "row";

% Inspect one link's inertial parameters
body = robot.Bodies{3};  % third link
inertial = body.Inertia; % [Ixx Iyy Izz Ixy Iyz Ixz]
mass = body.Mass;
com  = body.CenterOfMass; % [cx cy cz]

fprintf("Mass: %.3f\n", mass);
fprintf("CoM:  [%.3f %.3f %.3f]\n", com(1), com(2), com(3));
fprintf("Inertia (Ixx Iyy Izz Ixy Iyz Ixz):\n");
disp(inertial);

% Compute joint-space inertia at a configuration q
q = homeConfiguration(robot);
q_vec = [q.JointPosition];  % row vector
M = massMatrix(robot, q_vec);

% Basic checks: symmetry and positive definiteness
symErr = norm(M - M.');
eigM   = eig(M);

fprintf("M symmetry error: %.3e\n", symErr);
fprintf("Eigenvalues of M(q):\n");
disp(eigM);

% Simulink: use 'Rigid Body Tree' block to simulate dynamics
% - Set the 'Robot description' parameter to 'robot'
% - Connect joint torques and states to other Simulink blocks
      
