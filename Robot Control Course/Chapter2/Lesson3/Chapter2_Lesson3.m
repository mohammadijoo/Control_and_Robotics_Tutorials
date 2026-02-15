
% Create robot from URDF
robot = importrobot("my_robot.urdf");
robot.DataFormat = "row";
robot.Gravity = [0 0 -9.81];

% PD + gravity compensation
Kp = diag([80 80 80]);  % example for 3-DOF
Kd = diag([15 15 15]);

function tau = pd_grav_control(robot, q, dq, qd, dqd, Kp, Kd)
    e  = qd - q;
    de = dqd - dq;
    tau_pd = (Kp * e.' + Kd * de.').';   % row format
    tau_g  = gravityTorque(robot, q);
    tau    = tau_pd + tau_g;
end

% Inside a control loop:
% [q, dq] measured from hardware
% qd, dqd desired; call pd_grav_control and send tau.
