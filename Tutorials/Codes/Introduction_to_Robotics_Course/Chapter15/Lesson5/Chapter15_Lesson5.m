% Transition matrix P for [s1; s2; s3]
% Rows sum to 1. Example parameters (conceptual only):
alpha = 0.2;  % prob. worker in s1 moves to s2 (retrained)
beta  = 0.1;  % prob. worker in s1 moves to s3 (unemployed)
gamma = 0.1;  % prob. worker in s3 moves to s2 (re-employed)

P = [1 - alpha - beta, alpha,           beta;
     0.0,              0.9,             0.1;
     0.0,              gamma,           1 - gamma];

% Check row sums
rowSums = sum(P, 2);

% Compute eigenvalues and eigenvectors of P'
[V, D] = eig(P.');

% Find the eigenvector corresponding to eigenvalue 1
[~, idx] = min(abs(diag(D) - 1.0));
p_star = V(:, idx);
p_star = p_star / sum(p_star);  % normalize to sum to 1

disp('Stationary distribution p_star:');
disp(p_star);

unemployment_rate = p_star(3);
fprintf('Long-run unemployment probability: %.3f\n', unemployment_rate);
      
