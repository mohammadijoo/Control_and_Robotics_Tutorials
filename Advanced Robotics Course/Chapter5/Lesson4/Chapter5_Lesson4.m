function [feasible, T, X] = check_unicycle_feasibility(x0, v_fun, w_fun, ...
                                                       tspan, v_max, w_max)
% x0    : initial state [x0; y0; theta0]
% v_fun : handle @(t) v(t)
% w_fun : handle @(t) w(t)
% tspan : [t0 tf]
% v_max, w_max : bounds

    function dx = dyn(t, x)
        v = v_fun(t);
        w = w_fun(t);
        if abs(v) > v_max || abs(w) > w_max
            % Hard constraint violation: encode NaNs
            dx = [NaN; NaN; NaN];
            return;
        end
        dx = [v * cos(x(3));
              v * sin(x(3));
              w];
    end

    opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
    [T, X] = ode45(@dyn, tspan, x0, opts);

    if any(any(isnan(X)))
        feasible = false;
        return;
    end

    % Example workspace constraint
    if any(abs(X(:,1)) > 5.0) || any(abs(X(:,2)) > 5.0)
        feasible = false;
        return;
    end

    feasible = true;
end
      
