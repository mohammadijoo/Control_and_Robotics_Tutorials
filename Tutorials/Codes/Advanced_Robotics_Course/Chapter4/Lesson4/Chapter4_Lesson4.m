% Direct collocation for double integrator using fmincon
N   = 40;
T   = 1.0;
h   = T / N;
n_x = 2;
n_u = 1;

% Decision vector: [X(0)...X(N), U(0)...U(N-1)]
z0 = zeros((N + 1) * n_x + N * n_u, 1);

% Initial guess: straight-line position
X0 = zeros(N + 1, n_x);
for k = 1:(N + 1)
    t = (k - 1) * h;
    X0(k, 1) = t;
end
z0(1:(N + 1) * n_x) = X0(:);

% Objective
function J = objective(z, N, h, n_x, n_u)
    X = reshape(z(1:(N + 1) * n_x), N + 1, n_x);
    U = reshape(z((N + 1) * n_x + 1:end), N, n_u);
    J = 0.0;
    for k = 1:N
        uk = U(k, 1);
        J = J + h * uk * uk;
    end
end

% Nonlinear equality constraints: boundary + collocation
function [c, ceq] = constraints(z, N, h, n_x, n_u)
    X = reshape(z(1:(N + 1) * n_x), N + 1, n_x);
    U = reshape(z((N + 1) * n_x + 1:end), N, n_u);

    ceq = [];
    % initial state
    ceq = [ceq;
           X(1, 1) - 0.0;
           X(1, 2) - 0.0];
    % terminal state
    ceq = [ceq;
           X(end, 1) - 1.0;
           X(end, 2) - 0.0];

    % trapezoidal collocation
    for k = 1:N
        pk   = X(k, 1);
        vk   = X(k, 2);
        pkp1 = X(k + 1, 1);
        vkp1 = X(k + 1, 2);
        uk   = U(k, 1);

        fk_p   = vk;
        fk_v   = uk;
        fkp1_p = vkp1;
        fkp1_v = uk;

        ceq = [ceq;
               pkp1 - pk - 0.5 * h * (fk_p + fkp1_p);
               vkp1 - vk - 0.5 * h * (fk_v + fkp1_v)];
    end
    c = [];
end

options = optimoptions("fmincon", ...
    "Algorithm", "sqp", ...
    "MaxIterations", 200, ...
    "Display", "iter");

problem.objective = @(z) objective(z, N, h, n_x, n_u);
problem.x0       = z0;
problem.nonlcon  = @(z) constraints(z, N, h, n_x, n_u);
problem.lb       = [];
problem.ub       = [];
problem.options  = options;
problem.solver   = "fmincon";

[z_opt, J_opt] = fmincon(problem);

X_opt = reshape(z_opt(1:(N + 1) * n_x), N + 1, n_x);
U_opt = reshape(z_opt((N + 1) * n_x + 1:end), N, n_u);

t_grid = linspace(0, T, N + 1);
figure;
subplot(2, 1, 1);
plot(t_grid, X_opt(:, 1), "-o");
xlabel("t"); ylabel("p");
subplot(2, 1, 2);
stairs(t_grid(1:end-1), U_opt(:, 1));
xlabel("t"); ylabel("u");
      
