function numerical_oscillator_demo
    k = 10.0;
    y0 = [0.1; 0.0];
    tspan = [0 1];
    % ODE45 uses adaptive RK methods
    [t, y] = ode45(@(t,y) f_oscillator(t,y,k), tspan, y0);
    q_end = y(end,1);
    qdot_end = y(end,2);
    fprintf('q(T) = %g, qdot(T) = %g\n', q_end, qdot_end);
end

function dydt = f_oscillator(~, y, k)
    q = y(1);
    qdot = y(2);
    dqdt = qdot;
    dqdotdt = -k * q;
    dydt = [dqdt; dqdotdt];
end
      
