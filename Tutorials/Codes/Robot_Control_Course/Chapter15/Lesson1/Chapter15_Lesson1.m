
% Suppose we collected data from a baseline controller:
% qd_log: joint velocity samples, tau_log: measured actuator torques
% We estimate a simple polynomial friction model tau_f(qd) = a1*qd + a3*qd^3

coeffs = polyfit(qd_log, tau_log, 3);  % [a3 a2 a1 a0]

% Controller function (e.g., in a MATLAB Function block in Simulink)
function tau = joint_controller(q, qd, q_ref, qd_ref, qdd_ref, coeffs)
    Kp = 50; Kd = 10;
    q_tilde  = q  - q_ref;
    qd_tilde = qd - qd_ref;
    v = qdd_ref - Kd * qd_tilde - Kp * q_tilde;
    M = 1.2; C = 0; G = 0;

    tau_nom = M * v + C * qd + G;
    % Learned friction compensation
    a3 = coeffs(1); a2 = coeffs(2); a1 = coeffs(3); a0 = coeffs(4);
    tau_f_hat = a3*qd.^3 + a2*qd.^2 + a1*qd + a0;

    tau = tau_nom + tau_f_hat;
end
