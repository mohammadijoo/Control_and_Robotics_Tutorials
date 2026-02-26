Ts = 0.01; t = 0:Ts:2;
x_true = sin(2*pi*1*t);
sigma = 0.2;
y = x_true + sigma*randn(size(t));

% Quantize (8-bit, range [-1.5,1.5])
b = 8; ymin=-1.5; ymax=1.5;
Delta = (ymax-ymin)/2^b;
yq = ymin + Delta*round((y-ymin)/Delta);

% First-order low-pass
tau = 0.1;
alpha = Ts/(tau+Ts);
s_hat = zeros(size(yq));
for k = 2:length(yq)
    s_hat(k) = (1-alpha)*s_hat(k-1) + alpha*yq(k);
end

disp(Delta);
disp(var(s_hat - x_true));

% Simulink hint:
% Use blocks:
%   - "Band-Limited White Noise" to add noise
%   - "Zero-Order Hold" for sampling
%   - "Quantizer" for ADC
%   - "Discrete Transfer Fcn" for H(z) low-pass
