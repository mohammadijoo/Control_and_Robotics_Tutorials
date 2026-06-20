function traj_opt_double_integrator
    N = 40;
    T = 2.0;
    h = T / N;

    p0 = 0; v0 = 0;
    pf = 1; vf = 0;
    amax = 2.0;

    % Decision vector w: [p0 v0 a0 p1 v1 a1 ... a_{N-1} pN vN]'
    nz = 3 * N + 2;

    w0 = zeros(nz, 1);   % initial guess
    lb = -inf(nz, 1);
    ub = inf(nz, 1);

    % Fix initial state
    lb(1:2) = [p0; v0];
    ub(1:2) = [p0; v0];

    % Fix final state
    lb(end-1:end) = [pf; vf];
    ub(end-1:end) = [pf; vf];

    % Acceleration bounds
    for k = 0:N-1
        iu = 3 * k + 3;
        lb(iu) = -amax;
        ub(iu) =  amax;
    end

    opts = optimoptions("fmincon", "Display", "iter", ...
        "Algorithm", "sqp");

    w_opt = fmincon(@(w) cost_fun(w, N, h), w0, [], [], [], [], ...
                    lb, ub, @(w) dyn_constraints(w, N, h), opts);

    % Extract trajectory
    p = w_opt(1:3:end);
    v = w_opt(2:3:end);
    t = linspace(0, T, N+1);
    plot(t, p); xlabel("t"); ylabel("p(t)"); grid on;
end

function J = cost_fun(w, N, h)
    J = 0;
    for k = 0:N-1
        iu = 3 * k + 3;
        ak = w(iu);
        J = J + 0.5 * h * ak^2;
    end
end

function [c, ceq] = dyn_constraints(w, N, h)
    ceq = zeros(2 * N, 1);
    for k = 0:N-1
        ix = 3 * k + 1;
        iu = 3 * k + 3;

        pk = w(ix);
        vk = w(ix + 1);
        ak = w(iu);

        pkp1 = w(ix + 3);
        vkp1 = w(ix + 4);

        pNext = pk + h * vk;
        vNext = vk + h * ak;

        ceq(2 * k + 1) = pkp1 - pNext;
        ceq(2 * k + 2) = vkp1 - vNext;
    end
    c = [];
end
      
