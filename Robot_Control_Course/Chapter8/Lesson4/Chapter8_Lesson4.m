
% Example: PE check from logged regressor data Y_k (N x m)
load('regressor_log.mat', 'Y', 'dt');  % Y: N-by-m, dt: sampling time

[N, m] = size(Y);
G = zeros(m, m);
for k = 1:N
    phi_k = Y(k, :).';
    G = G + (phi_k * phi_k.');
end
G = dt * G;

lambda = eig((G + G.') / 2);    % symmetrized for numerical stability
lambda_min = min(lambda);

disp('Eigenvalues of G_N:');
disp(lambda.');
fprintf('lambda_min(G_N) = %.4f\n', lambda_min);

% Threshold for "practical PE" decision
lambda_threshold = 1e-2;
if lambda_min > lambda_threshold
    disp('Regressor is sufficiently exciting on this horizon.');
else
    disp('Regressor is poorly exciting; redesign trajectory.');
end
