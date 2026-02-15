
function tau = computed_torque(q, qd, q_d, qd_d, qdd_d, Kp, Kd, robot)
    % q, qd, ... are column vectors
    e  = q  - q_d;
    ed = qd - qd_d;

    v = qdd_d - Kd * ed - Kp * e;

    M = massMatrix(robot, q');
    C = coriolis(robot, q', qd');
    g = gravityTorque(robot, q');

    tau = M * v + C * qd + g';
end
