
function lab_compare_PD_CT
    T = 10.0;
    dt = 0.001;
    tspan = 0:dt:T;

    % Initial state [q1; q2; q1dot; q2dot]
    x0 = [0; 0; 0; 0];

    Kp = diag([100, 80]);
    Kd = diag([20, 16]);

    % Simulate PD
    [t_PD, x_PD] = ode45(@(t, x) dyn_closed_loop(t, x, Kp, Kd, false), tspan, x0);
    [Je_PD, Jtau_PD, emax_PD] = compute_metrics(t_PD, x_PD, Kp, Kd, false);

    % Simulate CT
    [t_CT, x_CT] = ode45(@(t, x) dyn_closed_loop(t, x, Kp, Kd, true), tspan, x0);
    [Je_CT, Jtau_CT, emax_CT] = compute_metrics(t_CT, x_CT, Kp, Kd, true);

    fprintf('PD: J_e = %.3f, J_tau = %.3f, e_max = %.3f\n', Je_PD, Jtau_PD, emax_PD);
    fprintf('CT: J_e = %.3f, J_tau = %.3f, e_max = %.3f\n', Je_CT, Jtau_CT, emax_CT);

    % Plot joint 1 tracking
    figure; hold on;
    plot(t_PD, qd(t_PD, 1), 'k', 'DisplayName', 'q1_d');
    plot(t_PD, x_PD(:,1), 'r', 'DisplayName', 'q1 PD');
    plot(t_CT, x_CT(:,1), 'b', 'DisplayName', 'q1 CT');
    xlabel('t [s]'); ylabel('q1 [rad]');
    legend show;
end

function xdot = dyn_closed_loop(t, x, Kp, Kd, useCT)
    q = x(1:2);
    qdot = x(3:4);

    % Reference
    [qd_vec, qd_dot_vec, qd_ddot_vec] = ref_traj(t);

    e = qd_vec - q;
    edot = qd_dot_vec - qdot;

    if useCT
        v = qd_ddot_vec + Kd*edot + Kp*e;
        M = M_matrix(q);
        C = C_matrix(q, qdot);
        g_vec = g_vector(q);
        tau = M*v + C*qdot + g_vec;
    else
        tau = Kp*e + Kd*edot;
    end

    M = M_matrix(q);
    C = C_matrix(q, qdot);
    g_vec = g_vector(q);

    qddot = M \ (tau - C*qdot - g_vec);

    xdot = [qdot; qddot];
end

function [Je, Jtau, emax] = compute_metrics(t, x, Kp, Kd, useCT)
    Je = 0; Jtau = 0; emax = 0;
    for k = 1:length(t)
        q = x(k,1:2)';
        qdot = x(k,3:4)';
        [qd_vec, qd_dot_vec, qd_ddot_vec] = ref_traj(t(k));
        e = qd_vec - q;
        edot = qd_dot_vec - qdot;
        if useCT
            v = qd_ddot_vec + Kd*edot + Kp*e;
            M = M_matrix(q);
            C = C_matrix(q, qdot);
            g_vec = g_vector(q);
            tau = M*v + C*qdot + g_vec;
        else
            tau = Kp*e + Kd*edot;
        end
        Je = Je + (e'*e) * (t(2)-t(1));
        Jtau = Je + (tau'*tau) * (t(2)-t(1));
        en = norm(e);
        if en > emax
            emax = en;
        end
    end
end

function [qd_vec, qd_dot_vec, qd_ddot_vec] = ref_traj(t)
    qd_vec = [0.5*sin(0.5*t); 0.5*cos(0.5*t)];
    qd_dot_vec = [0.25*cos(0.5*t); -0.25*sin(0.5*t)];
    qd_ddot_vec = [-0.125*sin(0.5*t); -0.125*cos(0.5*t)];
end

% M, C, g as in Python/C++ versions (2-DOF planar arm)
function M = M_matrix(q)
    m1 = 1.0; m2 = 1.0;
    l1 = 1.0; lc1 = 0.5; lc2 = 0.5; I1 = 0.1; I2 = 0.1;
    q1 = q(1); q2 = q(2);
    c2 = cos(q2);
    m11 = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2);
    m12 = I2 + m2*(lc2^2 + l1*lc2*c2);
    m22 = I2 + m2*lc2^2;
    M = [m11, m12; m12, m22];
end

function C = C_matrix(q, qdot)
    m2 = 1.0; l1 = 1.0; lc2 = 0.5;
    q2 = q(2);
    q1dot = qdot(1); q2dot = qdot(2);
    s2 = sin(q2);
    h = -m2*l1*lc2*s2;
    C = [h*q2dot, h*(q1dot + q2dot);
         -h*q1dot, 0];
end

function g_vec = g_vector(q)
    m1 = 1.0; m2 = 1.0;
    l1 = 1.0; lc1 = 0.5; lc2 = 0.5;
    g = 9.81;
    q1 = q(1); q2 = q(2);
    g1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2);
    g2 = m2*lc2*g*cos(q1 + q2);
    g_vec = [g1; g2];
end
