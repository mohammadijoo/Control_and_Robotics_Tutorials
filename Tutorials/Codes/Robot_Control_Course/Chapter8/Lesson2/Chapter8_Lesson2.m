
function adaptive_ct_1dof_sim()
    Lambda = 10.0;
    kD = 5.0;
    Gamma = diag([5.0, 1.0, 1.0]);

    theta_hat = [0.5; 0.1; 0.5];  % initial guess

    % True parameters (unknown to controller)
    theta_true = [2.0; 0.5; 3.0];

    T  = 10.0;
    dt = 1e-3;
    N  = round(T / dt);

    q  = 0.0;
    qd = 0.0;

    qs    = zeros(1, N);
    qds   = zeros(1, N);
    qd_ds = zeros(1, N);

    for k = 1:N
        t = (k - 1) * dt;

        % Desired trajectory: constant position
        q_d   = 0.5;
        qd_d  = 0.0;
        qdd_d = 0.0;

        tilde_q  = q - q_d;
        tilde_qd = qd - qd_d;

        qrd  = qd_d - Lambda * tilde_q;
        qrdd = qdd_d - Lambda * tilde_qd;
        s    = qd - qrd;

        Y = [qrdd, qrd, cos(q)];  % 1x3 row

        tau = Y * theta_hat + kD * s;

        % Plant dynamics: tau = theta_true' * [qdd; qd; cos(q)]
        a = theta_true(1);
        b = theta_true(2);
        c = theta_true(3);

        qdd = (tau - b * qd - c * cos(q)) / a;

        % Integrate
        qd = qd + qdd * dt;
        q  = q  + qd * dt;

        % Parameter update
        theta_dot = -Gamma * (Y.' * s);
        theta_hat = theta_hat + theta_dot * dt;

        qs(k)    = q;
        qds(k)   = qd;
        qd_ds(k) = q_d;
    end

    figure; plot((0:N-1) * dt, qs, 'LineWidth', 1.5); hold on;
    plot((0:N-1) * dt, qd_ds, '--', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('q [rad]');
    legend('q', 'q_d');
    title('Adaptive Computed-Torque Tracking (1-DOF)');
end
