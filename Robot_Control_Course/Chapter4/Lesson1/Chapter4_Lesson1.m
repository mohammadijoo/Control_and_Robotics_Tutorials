
function tau = joint_space_pd(q, qd, q_des, qd_des, Kp, Kd)
    % q, qd, q_des, qd_des: 2x1 vectors
    e_q  = q_des - q;
    e_qd = qd_des - qd;
    qdd_des = Kp * e_q + Kd * e_qd;
    tau = inverse_dynamics(q, qd, qdd_des);
end

function tau = task_space_pd(q, qd, x_des, xd_des, Kx, Dx, l1, l2)
    if nargin < 7
        l1 = 1.0; l2 = 1.0;
    end
    [x, J] = fk_and_jac_planar_2dof(q, l1, l2);
    xdot   = J * qd;

    e_x  = x_des  - x;
    e_xd = xd_des - xdot;

    xdd_des = Kx * e_x + Dx * e_xd;

    % Pseudoinverse (non-singular assumption)
    qdd_des = pinv(J) * xdd_des;
    tau = inverse_dynamics(q, qd, qdd_des);
end

function [x, J] = fk_and_jac_planar_2dof(q, l1, l2)
    q1 = q(1); q2 = q(2);
    x = [ l1*cos(q1) + l2*cos(q1+q2);
          l1*sin(q1) + l2*sin(q1+q2) ];
    J = [ -l1*sin(q1) - l2*sin(q1+q2), -l2*sin(q1+q2);
           l1*cos(q1) + l2*cos(q1+q2),  l2*cos(q1+q2) ];
end

function tau = inverse_dynamics(q, qd, qdd_des)
    % Placeholder for full dynamic model
    tau = qdd_des;
end
