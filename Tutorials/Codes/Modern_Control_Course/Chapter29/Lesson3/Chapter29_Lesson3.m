% Chapter29_Lesson3.m
%
% Numerical experiments for Lesson 3:
% Stability notions for continuous-time linear time-varying systems.
%
% Run in MATLAB or GNU Octave:
%     Chapter29_Lesson3

function Chapter29_Lesson3()
    fprintf('Scalar uniformly exponentially stable example\n');
    M = exp(0.5);
    alpha = 0.5;
    taus = [0, 1, 2, 5, 10];

    for tau = taus
        ratios = [];
        for t0 = linspace(0, 20, 41)
            phi = abs(phi_uniform_exponential_scalar(t0 + tau, t0));
            bound = M * exp(-alpha * tau);
            ratios(end + 1) = phi / bound;
        end
        fprintf('tau=%5.1f, max |Phi|/bound = %.6f\n', tau, max(ratios));
    end

    fprintf('\nUniformly stable but not uniformly attractive example\n');
    Ts = [1, 5, 20];
    t0s = [0, 10, 100, 1000, 10000];

    for T = Ts
        fprintf('T=%5.1f: ', T);
        for t0 = t0s
            fprintf('%.6f ', phi_uniform_stable_not_uniform_attractive(t0 + T, t0));
        end
        fprintf('\n');
    end

    fprintf('\n2x2 transition-matrix Frobenius norm estimates\n');
    for tau = [1, 2, 5, 10]
        norms = [];
        for t0 = linspace(0, 5, 6)
            Phi = rk4_phi(t0, t0 + tau, 0.005);
            norms(end + 1) = norm(Phi, 'fro');
        end
        fprintf('tau=%5.1f, max sampled ||Phi||_F = %.6f\n', tau, max(norms));
    end
end

function y = phi_uniform_exponential_scalar(t, t0)
    y = exp(-0.5 * (t - t0) + 0.25 * (cos(t0) - cos(t)));
end

function y = phi_uniform_stable_not_uniform_attractive(t, t0)
    y = (1.0 + t0) / (1.0 + t);
end

function A = A_of_t(t)
    k = 1.0 + 0.20 * sin(t);
    c = 0.80 + 0.10 * cos(2.0 * t);
    A = [0.0, 1.0; -k, -c];
end

function Phi = rk4_phi(t0, t1, h)
    if t1 < t0
        error('This routine assumes t1 >= t0.');
    end

    n = ceil((t1 - t0) / h);
    if n == 0
        Phi = eye(2);
        return;
    end

    h_eff = (t1 - t0) / n;
    t = t0;
    Phi = eye(2);

    for step = 1:n
        K1 = A_of_t(t) * Phi;
        K2 = A_of_t(t + 0.5 * h_eff) * (Phi + 0.5 * h_eff * K1);
        K3 = A_of_t(t + 0.5 * h_eff) * (Phi + 0.5 * h_eff * K2);
        K4 = A_of_t(t + h_eff) * (Phi + h_eff * K3);
        Phi = Phi + (h_eff / 6.0) * (K1 + 2.0 * K2 + 2.0 * K3 + K4);
        t = t + h_eff;
    end
end
