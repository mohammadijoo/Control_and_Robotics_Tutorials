% Chapter29_Lesson2.m
% Time-varying state transition matrix for x_dot = A(t) x.
% Includes a MATLAB computation and a Simulink-ready vectorized RHS.

clear; clc;

t0 = 0.0;
tf = 6.0;
n = 2;

Phi0 = eye(n);
y0 = Phi0(:);

opts = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);
[t, y] = ode45(@ltv_phi_rhs, [t0 tf], y0, opts);

Phi_ode = reshape(y(end, :), n, n);
disp('Phi(tf,t0) from MATLAB ode45 matrix IVP:');
disp(Phi_ode);

orders = [1 2 3 4 6 8];
for q = orders
    Phi_pb = peano_baker(t0, tf, q, 5000);
    err = norm(Phi_pb - Phi_ode, 'fro');
    fprintf('Peano-Baker order %2d error: %.6e\n', q, err);
end

Phi60 = transition_matrix_ode(0.0, 6.0);
Phi62 = transition_matrix_ode(2.5, 6.0);
Phi20 = transition_matrix_ode(0.0, 2.5);
fprintf('Composition error: %.6e\n', norm(Phi60 - Phi62 * Phi20, 'fro'));

% Simulink use:
% Put the ltv_phi_rhs equations inside a MATLAB Function block.
% Feed Clock time t and the 4-state vector PhiVec into the block.
% The output dPhiVec integrates through an Integrator block initialized by eye(2)(:).

function dy = ltv_phi_rhs(t, y)
    Phi = reshape(y, 2, 2);
    dPhi = A_matrix(t) * Phi;
    dy = dPhi(:);
end

function A = A_matrix(t)
    A = [0, 1;
        -2 - 0.5 * sin(t), -0.4 + 0.2 * cos(2 * t)];
end

function Phi = transition_matrix_ode(t0, tf)
    y0 = eye(2);
    y0 = y0(:);
    opts = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);
    [~, y] = ode45(@ltv_phi_rhs, [t0 tf], y0, opts);
    Phi = reshape(y(end, :), 2, 2);
end

function Phi = peano_baker(t0, tf, order, steps)
    h = (tf - t0) / steps;
    grid = linspace(t0, tf, steps + 1);

    terms = cell(order + 1, steps + 1);
    for j = 1:(steps + 1)
        terms{1, j} = eye(2);
    end

    for k = 2:(order + 1)
        terms{k, 1} = zeros(2);
        for j = 2:(steps + 1)
            smid = 0.5 * (grid(j - 1) + grid(j));
            integrand = A_matrix(smid) * terms{k - 1, j - 1};
            terms{k, j} = terms{k, j - 1} + h * integrand;
        end
    end

    Phi = zeros(2);
    for k = 1:(order + 1)
        Phi = Phi + terms{k, steps + 1};
    end
end
