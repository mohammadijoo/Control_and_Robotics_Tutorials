
function contact_transition_demo
    m = 1.0; g = 9.81; e = 0.2;
    x0 = [0.5; -1.0];  % initial [q; v]
    tspan = [0 3];

    opts = odeset('Events', @(t, x) impactEvent(t, x));
    [t, x] = ode45(@(t, x) dynamics(t, x, m, g), tspan, x0, opts);

    % If an impact occurs, apply reset and continue (simple loop)
    tAll = t; xAll = x;
    while t(end) < tspan(2)
        xImpact = x(end, :).';
        if xImpact(1) == 0 && xImpact(2) < 0
            xImpact(2) = -e * xImpact(2); % velocity reset
        end
        tspan2 = [t(end) tspan(2)];
        [t, x, te, xe, ie] = ode45(@(t, x) dynamics(t, x, m, g), ...
                                   tspan2, xImpact, opts);
        tAll = [tAll; t(2:end)];
        xAll = [xAll; x(2:end, :)];
        if isempty(te)
            break;
        end
    end

    figure;
    subplot(2, 1, 1); plot(tAll, xAll(:, 1)); ylabel('q');
    subplot(2, 1, 2); plot(tAll, xAll(:, 2)); ylabel('v'); xlabel('t');

end

function dx = dynamics(t, x, m, g)
    q = x(1); v = x(2);
    tau = pd_control(q, v, 0.2, 0.0, m, g);
    a = (tau - m * g) / m;
    dx = [v; a];
end

function [value, isterminal, direction] = impactEvent(t, x)
    q = x(1); v = x(2);
    value = q;          % detect q = 0
    isterminal = 1;     % stop integration
    direction = -1;     % only when crossing downward
end

function tau = pd_control(q, v, q_des, v_des, m, g)
    kp = 50; kd = 10;
    tau_g = m * g;
    tau_pd = -kp * (q - q_des) - kd * (v - v_des);
    tau = tau_g + tau_pd;
end
