dt = 0.1;
A = [1, dt;
     0, 1];
B = [0.5 * dt^2;
     dt];

n = 2; m = 1; N = 30;
W = diag([1e-4, 1e-4]);

mu0 = [0; 0];
Sigma0 = diag([0.05^2, 0.02^2]);

p_max = 2.0;
alpha_total = 0.05;
epsilon_k = alpha_total / N;
beta_eps = norminv(1 - epsilon_k, 0, 1);

% Precompute covariance sequence
Sigma = cell(N+1, 1);
Sigma{1} = Sigma0;
sigma_p = zeros(N+1, 1);
sigma_p(1) = sqrt(Sigma0(1, 1));
for k = 1:N
    Sigma{k+1} = A * Sigma{k} * A' + W;
    sigma_p(k+1) = sqrt(Sigma{k+1}(1, 1));
end

% Build condensed dynamics: X = Sx * x0 + Su * U
% Here we only need the mean position, so we could build linear maps for p_k.
% For clarity, we keep state dimension n=2.
Sx = zeros(n*(N+1), n);
Sx(1:n, :) = eye(n);
for k = 1:N
    Sx(k*n+1:(k+1)*n, :) = A^k;
end

Su = zeros(n*(N+1), m*N);
for k = 1:N
    for j = 1:k
        row_idx = k*n+1:(k+1)*n;
        col_idx = (j-1)*m+1:j*m;
        Su(row_idx, col_idx) = A^(k-j) * B;
    end
end

% Quadratic cost: minimize sum(u_k^2) + terminal position error
Q_u = 0.1 * eye(m*N);
Q_term = 10.0;
p_target = 1.5;

% Terminal position selector
E_pN = zeros(1, n*(N+1));
E_pN( (N)*n + 1 ) = 1;  % picks p_N

H = 2 * (Q_u + Q_term * (Su' * (E_pN' * E_pN) * Su));
f = 2 * Q_term * Su' * (E_pN' * (E_pN * Sx * mu0 - p_target));

% Chance constraints: mu_p_k + beta * sigma_p(k) <= p_max
Aineq = [];
bineq = [];

for k = 1:N
    E_pk = zeros(1, n*(N+1));
    E_pk(k*n + 1) = 1; % extract p_k
    row = E_pk * Su;
    offset = E_pk * Sx * mu0 + beta_eps * sigma_p(k+1);
    Aineq = [Aineq; row];
    bineq = [bineq; p_max - offset];
end

% Optional input bounds: -2 <= u_k <= 2
Aineq = [Aineq; eye(m*N); -eye(m*N)];
bineq = [bineq; 2*ones(m*N, 1); 2*ones(m*N, 1)];

options = optimoptions('quadprog','Display','off');
[U_opt, fval, exitflag] = quadprog(H, f, Aineq, bineq, [], [], [], [], [], options);

disp(['Exitflag: ', num2str(exitflag)]);
      
