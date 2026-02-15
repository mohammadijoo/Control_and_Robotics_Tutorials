
function [tau, theta_hat_next] = adaptive_ct_step(q, dq, qd, dqd, ddqd, ...
                                                  theta_hat, Kd, Lambda, Gamma_d, robot_model)
    % tracking error
    e  = q - qd;
    de = dq - dqd;

    % filtered error
    s = de + Lambda * e;

    % reference motion
    dqr  = dqd - Lambda * e;
    ddqr = ddqd - Lambda * de;

    % regressor matrix (n x p)
    Y = robot_model.regressor(q, dq, dqr, ddqr);

    % control torque
    tau = Y * theta_hat - Kd * s;

    % parameter update
    grad = Y.' * s;
    theta_hat_next = theta_hat - Gamma_d * grad;
end
