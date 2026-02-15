%% ---------------- Driver Layer (mock) ----------------
% raw sensor sample s_k
s_k = randn();

% calibration
alpha = 1.2; beta = -0.05;
y_k = alpha*s_k + beta;

% low-pass filtering in driver
lambda = 0.2;
persistent y_prev
if isempty(y_prev), y_prev = 0; end
y_k = (1-lambda)*y_prev + lambda*y_k;
y_prev = y_k;

%% ---------------- Middleware Layer (bus-like) ----------------
% Here we emulate middleware by packaging data into a struct.
msg.data = y_k;
msg.timestamp = tic;  % local timebase

%% ---------------- Application Layer ----------------
K = 0.8;
u = -K * msg.data;  % proportional control policy
fprintf('[APP] y=%.3f, u=%.3f\n', msg.data, u);
      
