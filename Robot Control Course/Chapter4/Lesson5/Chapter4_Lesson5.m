
function dx = planar2dof_opspace_ode(t, x, robot, Kp, Kd, x0, xf, T)

% x = [q1; q2; q1dot; q2dot]
q    = x(1:2);
qdot = x(3:4);

eeName = 'tool';
baseName = robot.BaseName;

% kinematics and Jacobian from rigidBodyTree
T_ee = getTransform(robot, q, eeName, baseName);
p = T_ee(1:2, 4);
J = geometricJacobian(robot, q, eeName);
J = J(1:2, 1:2); % planar subset

xd    = x_des(t, x0, xf, T);
xdDot = xdot_des(t, x0, xf, T);
xdd   = xddot_des(t, x0, xf, T);

e    = xd - p;
eDot = xdDot - J * qdot;

a_x = xdd + Kd * eDot + Kp * e;

% numerical Jdot
eps = 1e-6;
Jdot = zeros(2, 2);
for i = 1:2
    dq = zeros(2, 1);
    dq(i) = eps;
    Jp = geometricJacobian(robot, q + dq, eeName);
    Jm = geometricJacobian(robot, q - dq, eeName);
    Jp = Jp(1:2, 1:2);
    Jm = Jm(1:2, 1:2);
    Jdot = Jdot + (Jp - Jm) / (2 * eps) * qdot(i);
end

qdd_des = J \ (a_x - Jdot * qdot); % 2x2 solve

% inverse dynamics using rigidBodyTree
tau = inverseDynamics(robot, q, qdot, qdd_des);

M = massMatrix(robot, q);
C = velocityProduct(robot, q, qdot);
G = gravityTorque(robot, q);

qdd = M \ (tau - C - G);

dx = zeros(4,1);
dx(1:2) = qdot;
dx(3:4) = qdd;
end
