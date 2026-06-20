% Chapter17_Lesson4.m
% Noise Modeling in Sensors and Actuators for Dynamic Systems
% MATLAB script: state-space simulation with actuator colored noise,
% process noise, sensor bias drift, measurement noise, and quantization.

clear; clc; rng(17);

% Physical parameters
m = 1.2; c = 0.35; k = 4.0;
Ts = 0.01; N = 6000;
t = (0:N-1)' * Ts;

% Continuous-time matrices
A_c = [0 1; -k/m -c/m];
B_c = [0; 1/m];
G_c = [0; 1/m];
C = [1 0];

% Discretization (Euler for teaching/demo)
A = eye(2) + Ts*A_c;
B = Ts*B_c;
G = Ts*G_c;

% Noise parameters
sigma_w = 0.20;
sigma_v = 0.01;
sigma_bias_rw = 0.002;
delta_q = 0.001;
rho_a = 0.97;
sigma_a_ss = 0.08;

Qw = sigma_w^2;
Qa = sigma_a_ss^2;
Rv = sigma_v^2;
Rq = delta_q^2/12;

u_cmd = 0.8*sin(2*pi*0.7*t);

x = zeros(2,N);
y = zeros(N,1);
y_true = zeros(N,1);
Syy = zeros(N,1);

bias = 0;
aNoise = 0;
P = zeros(2,2);

for kIdx = 1:N-1
    % Colored actuator noise (AR(1))
    aNoise = rho_a*aNoise + sqrt(1-rho_a^2)*sigma_a_ss*randn;

    % Process noise
    w = sigma_w * randn;

    % Plant update
    u_actual = u_cmd(kIdx) + aNoise;
    x(:,kIdx+1) = A*x(:,kIdx) + B*u_actual + G*w;

    % Sensor bias random walk
    bias = bias + sigma_bias_rw*sqrt(Ts)*randn;

    % White measurement noise + quantization
    v = sigma_v * randn;
    y_analog = C*x(:,kIdx+1) + bias + v;
    q = delta_q*round(y_analog/delta_q) - y_analog;
    y(kIdx+1) = y_analog + q;
    y_true(kIdx+1) = C*x(:,kIdx+1);

    % Covariance recursion (independent noises)
    P = A*P*A' + G*Qw*G' + B*Qa*B';
    Syy(kIdx+1) = C*P*C' + Rv + Rq;
end

burn = 1000;
idx = burn:N;
empVarY = var(y(idx),1);
empVarYTrue = var(y_true(idx),1);
predVarY = mean(Syy(idx));

fprintf('=== Noise Modeling Demo (MATLAB) ===\n');
fprintf('Empirical var(y_true): %.6e\n', empVarYTrue);
fprintf('Empirical var(y)    : %.6e\n', empVarY);
fprintf('Predicted mean Syy  : %.6e (bias RW excluded)\n', predVarY);
fprintf('Quantization theory : %.6e\n', delta_q^2/12);

% Simple PSD estimate (periodogram)
yc = y(idx) - mean(y(idx));
Y = fft(yc);
L = length(yc);
f = (0:floor(L/2))'/(L*Ts);
Pyy = (abs(Y(1:floor(L/2)+1)).^2)/(L/Ts);

[~, order] = sort(Pyy, 'descend');
fprintf('\nTop spectral peaks (Hz, PSD):\n');
for i = 1:5
    ii = order(i);
    fprintf('%8.3f Hz, %.6e\n', f(ii), Pyy(ii));
end

% Simulink implementation notes (manual setup):
% 1) Use a State-Space block with A_c, B_c, C, D=0.
% 2) Add Band-Limited White Noise for process force and Sum into actuator/plant input.
% 3) Add a Discrete Filter block for actuator AR(1) noise (z-domain: 1/(1-rho_a z^-1)).
% 4) Add a Unit Delay + Random Number to create bias random walk.
% 5) Add Quantizer block with interval delta_q.
% 6) Scope measured y and true position for comparison.
