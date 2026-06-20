
function u_safe = joint_cbf_qp_1dof(q, qdot, q_ref, params)
    % params: struct with fields q_min, q_max, gamma, u_min, u_max, kp, kd

    q_min = params.q_min;
    q_max = params.q_max;
    gamma = params.gamma;
    u_min = params.u_min;
    u_max = params.u_max;
    kp = params.kp;
    kd = params.kd;

    % Nominal PD velocity
    e = q_ref - q;
    edot = -qdot;
    u_nom = kp * e + kd * edot;

    % CBF bounds
    cbf_lower = -gamma * (q - q_min);  % u >= cbf_lower
    cbf_upper =  gamma * (q_max - q);  % u <= cbf_upper

    % QP: min (u - u_nom)^2
    H = 2;           % scalar (2 * 1)
    f = -2 * u_nom;  % so objective = u^2 - 2 u_nom u + const

    % Inequalities A u <= b
    A = [ 1;   % u <= cbf_upper
         -1]; % -u <= -cbf_lower  (u >= cbf_lower)
    b = [cbf_upper; -cbf_lower];

    % Bounds
    lb = u_min;
    ub = u_max;

    options = optimoptions('quadprog', 'Display', 'off');
    [u_opt,~,exitflag] = quadprog(H, f, A, b, [], [], lb, ub, [], options);

    if exitflag <= 0
        u_safe = max(u_min, min(u_max, u_nom));
    else
        u_safe = u_opt;
    end
end
