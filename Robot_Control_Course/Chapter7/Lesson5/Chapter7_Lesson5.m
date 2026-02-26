
function robust_tracking_lab_matlab
    m = 1.0;
    l = 0.5;
    g = 9.81;
    I = m * l^2;

    lam = 4.0;
    k_s = 5.0;
    phi = 0.1;
    alpha = 30.0;

    x0 = [0.0; 0.0; 0.0];  % [q; dq; d_hat]
    tf = 20.0;

    [t, x] = ode45(@(t,x) dyn(t, x, m, l, g, I, lam, k_s, phi, alpha), [0 tf], x0);

    q = x(:,1);
    dq = x(:,2);
    d_hat = x(:,3);

    qd = q_d(t);
    dqd = dq_d(t);
    e = q - qd;
    de = dq - dqd;
    s = de + lam * e;

    figure;
    plot(t, qd, t, q, '--');
    xlabel('t [s]'); ylabel('q [rad]');
    legend('q_d', 'q');
    title('Robust tracking with disturbance observer');

    figure;
    plot(t, e, t, s);
    legend('e', 's');
    xlabel('t [s]'); ylabel('error');
    title('Tracking error and sliding variable');

    figure;
    plot(t, d_hat);
    xlabel('t [s]'); ylabel('d\_hat');
    title('Disturbance estimate');

end

function y = q_d(t)
    y = 0.5 * sin(t);
end

function y = dq_d(t)
    y = 0.5 * cos(t);
end

function y = ddq_d(t)
    y = -0.5 * sin(t);
end

function d = disturbance(t, q, dq)
    sign_dq = zeros(size(dq));
    sign_dq(dq > 0) = 1;
    sign_dq(dq < 0) = -1;
    d = 1.5 * sin(3.0 * t) + 0.5 * sign_dq;
end

function dx = dyn(t, x, m, l, g, I, lam, k_s, phi, alpha)
    q = x(1);
    dq = x(2);
    d_hat = x(3);

    e = q - q_d(t);
    de = dq - dq_d(t);
    s = de + lam * e;

    g_term = m * g * l * sin(q);

    tau_nom = I * (ddq_d(t) - lam * de) + g_term;

    sigma = s / phi;
    if sigma > 1
        sat_s = 1;
    elseif sigma < -1
        sat_s = -1;
    else
        sat_s = sigma;
    end
    tau_rob = -k_s * sat_s;

    tau = tau_nom + tau_rob - d_hat;

    d = disturbance(t, q, dq);
    ddq = (tau + d - g_term) / I;

    q_ddot_est = ddq;
    d_hat_dot = -alpha * d_hat + alpha * (I * q_ddot_est + g_term - tau);

    dx = [dq; ddq; d_hat_dot];
end
