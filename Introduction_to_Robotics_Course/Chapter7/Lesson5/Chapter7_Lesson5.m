fs = 100; Ts = 1/fs; N = 200;
t = (0:N-1)*Ts;

x = sin(2*pi*3*t);
sigma_n = 0.05;
sigma_b = 1e-4;

% Noise
n = sigma_n*randn(size(t));

% Drift as random walk
b = zeros(size(t));
for k=2:N
    b(k) = b(k-1) + sigma_b*randn();
end

y = x + n + b;

% Quantization
B = 10; ymin=-2; ymax=2;
Delta = (ymax-ymin)/2^B;
yq = Delta*round(y/Delta);

% Compare measured quantization variance to Delta^2/12
eq = yq - y;
fprintf('Delta=%g, Var(eq)=%g, Theoretical=%g\n', Delta, var(eq), Delta^2/12);

% Bias estimate via mean on first 20 samples
b_hat = mean(yq(1:20));
y_comp = yq - b_hat;
