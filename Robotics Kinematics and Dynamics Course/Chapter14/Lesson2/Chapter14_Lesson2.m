function lesson14_gear_dynamics_demo()
    % Parameters
    Jm = 0.002;
    Jg = 0.001;
    JL = 0.05;
    bm = 0.001;
    bq = 0.02;
    n  = 60;
    eta = 0.9;
    tau_m = 1.0;
    tau_ext = 0.0;
    k_load = 5.0;

    J_eq = n^2 * (Jm + Jg) + JL;
    b_eq = n^2 * bm + bq;

    params = struct('J_eq', J_eq, 'b_eq', b_eq, ...
                    'k_load', k_load, 'n', n, ...
                    'eta', eta, 'tau_m', tau_m, ...
                    'tau_ext', tau_ext);

    tspan = [0 1];
    x0 = [0; 0]; % [q; qdot]
    [t, x] = ode45(@(t, x) joint_ode(t, x, params), tspan, x0);

    q = x(:, 1);
    figure; plot(t, q); xlabel('t [s]'); ylabel('q [rad]');
    title('Geared joint response');

    fprintf('Equivalent inertia J_eq = %.4f\n', J_eq);
end

function xdot = joint_ode(t, x, p)
    q = x(1);
    qdot = x(2);

    rhs = p.n * p.eta * p.tau_m + p.tau_ext ...
          - p.b_eq * qdot - p.k_load * q;

    qddot = rhs / p.J_eq;
    xdot = [qdot; qddot];
end
      
