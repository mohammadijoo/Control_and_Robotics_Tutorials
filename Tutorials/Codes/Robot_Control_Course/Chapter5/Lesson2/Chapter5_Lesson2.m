
function [tau, lambda] = projectedTorque(q, qd, q_ref, qd_ref, qdd_ref)
% Equality-constrained torque projection for a manipulator.
%
% q, qd      : current joint position and velocity (n-by-1)
% q_ref, ... : reference trajectory
%
% Dependencies:
%   getMassMatrixAndBias   - returns M(q), h(q, qd)
%   getConstraintJacobian  - returns Jc(q), bc(q, qd, t)

    % Dynamics
    [M, h] = getMassMatrixAndBias(q, qd);
    [Jc, bc] = getConstraintJacobian(q, qd);

    n = length(q);
    m = size(Jc, 1);

    % Unconstrained computed-torque PD
    Kp = diag(100 * ones(n, 1));
    Kd = diag(20  * ones(n, 1));

    e  = q_ref  - q;
    ed = qd_ref - qd;
    v  = qdd_ref + Kd * ed + Kp * e;

    tau0 = M * v + h;

    % Projection
    Minv_tau0_minus_h = M \ (tau0 - h);
    Minv_JcT = M \ Jc';

    JMJT = Jc * Minv_JcT;
    eps = 1e-9;
    Lambda_c = inv(JMJT + eps * eye(m));

    lambda = -Lambda_c * (Jc * Minv_tau0_minus_h + bc);
    tau = tau0 + Jc' * lambda;
end
