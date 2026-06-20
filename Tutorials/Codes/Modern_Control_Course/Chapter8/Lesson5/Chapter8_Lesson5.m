% Chapter8_Lesson5.m
%
% Numerical computation of the state transition matrix Phi(t)=expm(A*t).
%
% Demonstrates:
% 1. MATLAB expm for dense matrix exponentials.
% 2. Educational scaling-and-squaring Taylor implementation.
% 3. Exact continuous-to-discrete conversion using a Van Loan block matrix.
% 4. Semigroup and inverse verification tests.

clear; clc;

A = [ 0  1  0;
      0  0  1;
     -2 -3 -4 ];

B = [0; 0; 1];

t = 0.5;
s = 0.25;
h = 0.1;

Phi_builtin = expm(A*t);
Phi_scratch = local_expm_taylor_scaling_squaring(A*t, 45);

disp('A =');
disp(A);

disp('Phi(t) from MATLAB expm =');
disp(Phi_builtin);

disp('Phi(t) from scratch scaling-and-squaring Taylor =');
disp(Phi_scratch);

disp('Infinity-norm difference =');
disp(norm(Phi_builtin - Phi_scratch, inf));

semigroup_error = norm(expm(A*t)*expm(A*s) - expm(A*(t+s)), inf);
disp('Semigroup error ||Phi(t)Phi(s)-Phi(t+s)||_inf =');
disp(semigroup_error);

inverse_error = norm(expm(A*t)*expm(-A*t) - eye(size(A)), inf);
disp('Inverse error ||Phi(t)Phi(-t)-I||_inf =');
disp(inverse_error);

% Exact zero-order-hold discretization:
% x_{k+1} = Ad x_k + Bd u_k
n = size(A,1);
m = size(B,2);
M = [A B; zeros(m,n) zeros(m,m)];
EM = expm(M*h);
Ad = EM(1:n, 1:n);
Bd = EM(1:n, n+1:n+m);

disp('Exact ZOH discretization with h=0.1:');
disp('Ad =');
disp(Ad);
disp('Bd =');
disp(Bd);

% If the Control System Toolbox is available, c2d gives the same result.
if exist('ss', 'file') == 2 && exist('c2d', 'file') == 2
    C = eye(n);
    D = zeros(n,m);
    sysc = ss(A,B,C,D);
    sysd = c2d(sysc, h, 'zoh');
    disp('Control System Toolbox comparison: norm(sysd.A - Ad, inf) =');
    disp(norm(sysd.A - Ad, inf));
end

function E = local_expm_taylor_scaling_squaring(M, order)
    n = size(M,1);
    mnorm = norm(M, inf);
    if mnorm > 0
        scale = max(0, ceil(log2(mnorm)));
    else
        scale = 0;
    end

    A_scaled = M / 2^scale;
    E = eye(n);
    term = eye(n);

    for k = 1:order
        term = term * A_scaled / k;
        E = E + term;
    end

    for i = 1:scale
        E = E * E;
    end
end
