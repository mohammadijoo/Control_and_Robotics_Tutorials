function collocation_two_link_lab
    N = 40;
    T = 2.0;
    h = T / N;
    nx = 4; nu = 2;

    nvar = (N+1) * (nx + nu);

    % Initial guess: rest configuration, zero control
    x_init = [0; 0; 0; 0];
    x_goal = [pi/2; 0; 0; 0];

    z0 = zeros(nvar, 1);
    for k = 0:N
        offset = k * (nx + nu);
        z0(offset + (1:nx)) = x_init;
        z0(offset + nx + (1:nu)) = [0; 0];
    end

    % Variable bounds
    q_min = -pi; q_max = pi;
    v_max = 4.0;
    u_max = 10.0;
    lb = -inf(nvar, 1);
    ub =  inf(nvar, 1);
    for k = 0:N
        offset = k * (nx + nu);
        % q1, q2
        lb(offset + 1) = q_min; ub(offset + 1) = q_max;
        lb(offset + 2) = q_min; ub(offset + 2) = q_max;
        % v1, v2
        lb(offset + 3) = -v_max; ub(offset + 3) = v_max;
        lb(offset + 4) = -v_max; ub(offset + 4) = v_max;
        % u1, u2
        lb(offset + 5) = -u_max; ub(offset + 5) = u_max;
        lb(offset + 6) = -u_max; ub(offset + 6) = u_max;
    end

    % Objective and constraints as nested functions
    fun = @(z)objective_fun(z, N, h, nx, nu);
    nonlcon = @(z)collocation_constraints(z, N, h, nx, nu, x_init, x_goal);

    options = optimoptions("fmincon", ...
        "Algorithm", "interior-point", ...
        "SpecifyObjectiveGradient", false, ...
        "SpecifyConstraintGradient", false, ...
        "MaxIterations", 200);

    [z_star, ~] = fmincon(fun, z0, [], [], [], [], lb, ub, nonlcon, options);

    % Extract optimal states and controls
    X = zeros(nx, N+1);
    U = zeros(nu, N+1);
    for k = 0:N
        offset = k * (nx + nu);
        X(:, k+1) = z_star(offset + (1:nx));
        U(:, k+1) = z_star(offset + nx + (1:nu));
    end

    t_grid = linspace(0, T, N+1);
    figure; plot(t_grid, X(1,:), t_grid, X(2,:));
    xlabel("time [s]"); ylabel("joint angles [rad]");
    legend("q1", "q2"); grid on;

    % Export U to workspace for Simulink block "From Workspace"
    assignin("base", "torque_sequence", [t_grid.' U.']);
end

function J = objective_fun(z, N, h, nx, nu)
    J = 0;
    for k = 0:N
        offset = k * (nx + nu);
        u = z(offset + nx + (1:nu));
        J = J + 0.5 * h * (u.' * u);
    end
end

function [c, ceq] = collocation_constraints(z, N, h, nx, nu, x_init, x_goal)
    c = []; % no inequality constraints here
    ceq = [];

    % Boundary constraints
    x0 = z(1:nx);
    xN = z(N * (nx + nu) + (1:nx));
    ceq = [ceq;
           x0 - x_init;
           xN - x_goal];

    % Collocation constraints
    for k = 0:(N-1)
        offk  = k * (nx + nu);
        offk1 = (k+1) * (nx + nu);

        xk  = z(offk  + (1:nx));
        uk  = z(offk  + nx + (1:nu));
        xk1 = z(offk1 + (1:nx));
        uk1 = z(offk1 + nx + (1:nu));

        fk  = two_link_dynamics(xk,  uk);
        fk1 = two_link_dynamics(xk1, uk1);

        defect = xk1 - xk - 0.5 * h * (fk + fk1);
        ceq = [ceq; defect];
    end
end

function xdot = two_link_dynamics(x, u)
    q1 = x(1); q2 = x(2);
    v1 = x(3); v2 = x(4);
    u1 = u(1); u2 = u(2);

    m1 = 1.0; m2 = 1.0;
    l1 = 1.0; l2 = 1.0;
    lc1 = 0.5; lc2 = 0.5;
    I1 = 0.12; I2 = 0.12; g = 9.81;

    s2 = sin(q2); c2 = cos(q2);

    M11 = I1 + I2 + m2 * l1^2 + 2 * m2 * l1 * lc2 * c2;
    M12 = I2 + m2 * l1 * lc2 * c2;
    M21 = M12;
    M22 = I2;
    M = [M11, M12; M21, M22];

    hC = m2 * l1 * lc2 * s2;
    C = [0,  -2 * hC * v2;
         hC * v1, 0];

    g1 = (m1 * lc1 + m2 * l1) * g * cos(q1) + m2 * lc2 * g * cos(q1 + q2);
    g2 = m2 * lc2 * g * cos(q1 + q2);
    G = [g1; g2];

    v = [v1; v2];
    a = M \ (u - C * v - G);

    xdot = [v; a];
end
      
