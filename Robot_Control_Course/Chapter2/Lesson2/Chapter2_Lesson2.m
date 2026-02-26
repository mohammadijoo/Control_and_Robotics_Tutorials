
function joint_pd_demo()
    J = 0.5;
    b = 0.05;
    qd = 1.0;

    zeta = 0.7;
    omega_n = 6.0;
    Kp = J * omega_n^2;
    Kd = 2 * J * zeta * omega_n - b;

    x0 = [0.0; 0.0];  % [q; dq]
    tspan = [0 2];
    [t, x] = ode45(@(t, x) dynamics_pd(t, x, qd, J, b, Kp, Kd), tspan, x0);

    q = x(:, 1);
    figure;
    plot(t, q, "LineWidth", 1.5); hold on;
    plot(t, qd * ones(size(t)), "--", "LineWidth", 1.0);
    xlabel("time [s]");
    ylabel("joint position [rad]");
    legend("q(t)", "qd");
    grid on;
end

function dx = dynamics_pd(t, x, qd, J, b, Kp, Kd)
    q = x(1);
    dq = x(2);

    e = qd - q;
    de = -dq; % since qd is constant

    tau = Kp * e + Kd * de;

    ddq = (tau - b * dq) / J;
    dx = [dq; ddq];
end
