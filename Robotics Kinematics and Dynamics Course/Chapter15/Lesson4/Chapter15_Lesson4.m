function dy = baumgarte_mass2d(t, y, params)
    % y = [x; y; vx; vy]
    x   = y(1);
    y_p = y(2);
    vx  = y(3);
    vy  = y(4);

    m     = params.m;
    g     = params.g;
    L     = params.L;
    alpha = params.alpha;
    beta  = params.beta;

    M = m * eye(2);
    h = [0.0; m * g];

    phi = x^2 + y_p^2 - L^2;

    J    = [2 * x, 2 * y_p];      % 1x2
    Jdot = [2 * vx, 2 * vy];      % 1x2
    qdot = [vx; vy];

    % Build K and rhs
    K = zeros(3,3);
    K(1:2,1:2) = M;
    K(1:2,3)   = -J.';
    K(3,1:2)   = J;

    rhs_dyn = -h;
    rhs_con = - (Jdot * qdot ...
                 + 2 * alpha * (J * qdot) ...
                 + beta^2 * phi);
    rhs = [rhs_dyn; rhs_con];

    sol = K \ rhs;
    qddot = sol(1:2);

    dy = [vx; vy; qddot];
end

% Example usage:
params.m = 1.0;
params.g = 9.81;
params.L = 1.0;
params.alpha = 5.0;
params.beta  = 10.0;

y0 = [params.L * 1.01; 0; 0; 0];
[t, Y] = ode45(@(t,y) baumgarte_mass2d(t,y,params), [0 5], y0);

phi_vals = Y(:,1).^2 + Y(:,2).^2 - params.L^2;
max_phi = max(abs(phi_vals))
      
