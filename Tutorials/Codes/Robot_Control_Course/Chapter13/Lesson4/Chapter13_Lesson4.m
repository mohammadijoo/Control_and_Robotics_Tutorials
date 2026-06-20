
function safe_control_demo
    % Parameters
    x0    = 0.1;
    u_min = -2.0;
    u_max =  2.0;
    kappa = 5.0;
    k_p   = 5.0;
    T     = 5.0;
    dt    = 1e-3;

    N = floor(T / dt);
    x = x0;
    x_ref = 1.0;

    t_vec = zeros(N+1,1);
    x_vec = zeros(N+1,1);
    u_nom_vec  = zeros(N,1);
    u_safe_vec = zeros(N,1);

    t_vec(1) = 0.0;
    x_vec(1) = x0;

    for k = 1:N
        t = k * dt;
        u_des = -k_p * (x - x_ref);
        u_safe = safe_input(x, u_des, u_min, u_max, kappa);
        x = x + dt * u_safe;

        t_vec(k+1) = t;
        x_vec(k+1) = x;
        u_nom_vec(k)  = u_des;
        u_safe_vec(k) = u_safe;
    end

    % Plot results
    figure;
    subplot(2,1,1);
    plot(t_vec, x_vec); hold on;
    yline(0, '--');
    ylabel('x(t)');
    title('State with safety filter and input limits');

    subplot(2,1,2);
    plot(t_vec(2:end), u_nom_vec, 'DisplayName','u\_des'); hold on;
    plot(t_vec(2:end), u_safe_vec, '--', 'DisplayName','u\_safe');
    yline(u_min, ':', 'DisplayName','u\_min');
    yline(u_max, ':', 'DisplayName','u\_max');
    xlabel('time');
    ylabel('input');
    legend('Location','best');
end

function u_safe = safe_input(x, u_des, u_min, u_max, kappa)
    % Scalar safety filter for dot{x} = u with h(x) = x.
    u_low = max(u_min, -kappa * x);
    u_high = u_max;
    u_safe = min(u_high, max(u_low, u_des));
end
