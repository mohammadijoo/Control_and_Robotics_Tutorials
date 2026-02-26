% samples.csv contains a column x
x = readmatrix("samples.csv");
N = length(x);

mu_hat = mean(x);
sigma_hat = std(x);

% expected values
mu0 = 0.0;
sigma0 = 0.02;

% z-score for bias test
z = (mu_hat - mu0) / (sigma0 / sqrt(N));

fprintf("mu_hat=%f, sigma_hat=%f, z=%f\n", mu_hat, sigma_hat, z);

if abs(z) > 3
    warning("Bias suspected beyond 3-sigma bound.");
end

% Simple step-response fit: y(t)=K(1-exp(-t/tau))
t = (0:N-1)' * 0.01;  % Ts=10ms
y = x;
K_hat = max(y);
idx63 = find(y >= 0.63*K_hat, 1);
tau_hat = t(idx63);

fprintf("Estimated tau ~ %f s\n", tau_hat);
