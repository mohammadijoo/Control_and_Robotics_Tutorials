function lesson3_matrix_exponential_demo()
    A = [0 1; -1 0];
    t = 0.7;

    % Library
    E_lib = expm(A*t);

    % Series (pedagogical)
    tol = 1e-14; maxTerms = 400;
    [E_series, used] = expm_series(A, t, tol, maxTerms);

    disp("Terms used: " + used);
    disp("expm(A*t) (MATLAB):"); disp(E_lib);
    disp("series exp(A*t):"); disp(E_series);
    disp("Fro error:"); disp(norm(E_lib - E_series, 'fro'));

    % Optional Simulink validation idea:
    % Simulate xdot = A x with x(0)=x0 and compare x(t)=expm(A t)*x0
    x0 = [1; 0];
    x_exact = E_lib * x0;
    disp("Exact x(t) from expm:"); disp(x_exact);
end

function [S, used] = expm_series(A, t, tol, maxTerms)
    n = size(A,1);
    S = eye(n);
    term = eye(n);
    At = A*t;

    used = maxTerms;
    for k = 1:maxTerms
        term = (term*At) / k;
        Snew = S + term;
        if norm(term,'fro') <= tol * norm(Snew,'fro')
            S = Snew;
            used = k;
            return;
        end
        S = Snew;
    end
end
