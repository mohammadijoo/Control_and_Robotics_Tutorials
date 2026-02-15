function cost_rate_age_replacement_demo
    mtbf = 2000;     % hours
    lam = 1/mtbf;
    C_p = 200;       % preventive replacement cost
    C_f = 800;       % corrective replacement cost

    tau_vec = linspace(100, 3000, 50);
    g_vec = arrayfun(@(tau) cost_rate_age_replacement(lam, C_p, C_f, tau), tau_vec);

    [g_star, idx] = min(g_vec);
    tau_star = tau_vec(idx);

    fprintf("Approx optimal replacement age: %.1f hours\n", tau_star);
    fprintf("Min cost rate: %.2f cost units per hour\n", g_star);

    figure;
    plot(tau_vec, g_vec, 'LineWidth', 1.5);
    xlabel('Replacement age tau [hours]');
    ylabel('Cost rate g(tau) [cost units/hour]');
    grid on;
end

function g = cost_rate_age_replacement(lam, C_p, C_f, tau)
    if tau <= 0
        error('tau must be positive');
    end
    num = C_f + (C_p - C_f) * exp(-lam * tau);
    den = 1 - exp(-lam * tau);
    g = lam * num / den;
end
      
