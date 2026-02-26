
function tau = smc_torque(q, dq, qd, dqd, ddqd, Lambda, K, phi)
% Sliding-mode control for an n-DOF manipulator.
% q, dq, qd, dqd, ddqd are column vectors (n x 1).

    % Tracking error
    e    = q  - qd;
    eDot = dq - dqd;

    % Sliding variable
    s = eDot + Lambda * e;

    % Robot model functions (user-defined, e.g., from Robotics System Toolbox)
    M = M_robot(q);       % n x n
    C = C_robot(q, dq);   % n x n
    g = g_robot(q);       % n x 1

    % Equivalent control
    tau_eq = M * (ddqd - Lambda * eDot) + C * dq + g;

    % Boundary-layer saturation using tanh
    phi_safe = max(phi, 1e-6);
    sat_arg = s ./ phi_safe;
    sat_val = tanh(sat_arg);  % approximate sat(s/phi)

    tau_sw = -K * sat_val;

    tau = tau_eq + tau_sw;
end
