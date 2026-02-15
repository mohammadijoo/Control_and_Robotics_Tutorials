
function tau = fl_controller(q, qdot, qd, qd_dot, qd_ddot, Kp, Kv, robot)
% q, qdot, qd, qd_dot, qd_ddot: column vectors
% Kp, Kv: diagonal or full gain matrices
% robot: rigidBodyTree from Robotics System Toolbox

e = q - qd;
e_dot = qdot - qd_dot;

v = qd_ddot - Kv * e_dot - Kp * e;

% Compute dynamics using Robotics System Toolbox:
M = massMatrix(robot, q.');
% velocityProduct(robot,q,qdot) returns C(q,qdot)qdot
cq = velocityProduct(robot, q.', qdot.');
gq = gravityTorque(robot, q.');

tau = M * v + cq.' + gq.';
end
