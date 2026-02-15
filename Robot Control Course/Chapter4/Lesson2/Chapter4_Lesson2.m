
function tau = opSpaceControlStep(q, qd, x, xd, x_d, xd_d, xdd_d, Kp, Kd, robot)
% OPSPACECONTROLSTEP  One control step of operational-space control.
%
% Inputs:
%   q, qd   : joint position and velocity (n x 1)
%   x, xd   : current task position and velocity (m x 1)
%   x_d, xd_d, xdd_d : desired task trajectory signals (m x 1)
%   Kp, Kd  : task-space gains (m x m)
%   robot   : rigidBodyTree model (Robotics System Toolbox)
%
% Output:
%   tau     : joint torques (n x 1)

    % Errors
    e  = x  - x_d;
    ed = xd - xd_d;

    % Desired task acceleration
    xdd_ref = xdd_d - Kd * ed - Kp * e;

    % Joint-space inertia and nonlinear terms
    M = massMatrix(robot, q');
    % generalized forces from inverseDynamics with zero acceleration
    h = inverseDynamics(robot, q', qd', zeros(size(q')));

    % End-effector Jacobian and its derivative (approx via finite difference
    % or robot-specific routine)
    eeName = robot.BodyNames{end};
    J = geometricJacobian(robot, q', eeName); % 6 x n for full spatial
    % For a pure position task, select top 3 rows:
    Jpos = J(1:3, :);

    % Approximate Jdot * qd via numerical differentiation or compute analytically
    Jdot_qd = zeros(size(Jpos, 1), 1); % placeholder

    Minv = inv(M);
    JMJT = Jpos * Minv * Jpos';
    Lambda = inv(JMJT);

    mu = Lambda * (Jpos * Minv * h) - Lambda * Jdot_qd;
    p  = zeros(size(mu)); % if gravity already in h

    F = Lambda * xdd_ref + mu + p;

    tau = Jpos' * F;
end
