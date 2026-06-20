% Chapter17_Lesson3.m
% Response of Linear Systems to Random Inputs: Mean and Variance Propagation
% MATLAB/Simulink-oriented implementation (moment equations + Monte Carlo)

clear; clc; rng(10);

% Continuous-time mass-spring-damper
m = 1.0; c = 0.5; k = 3.0;
A = [0 1; -k/m -c/m];
B = [0; 1/m];          % deterministic mean-force channel
Gamma = [0; 1/m];      % white-noise channel
C = [1 0];

u_mean = 1.0;
q = 0.7;               % white-noise spectral intensity

% Time integration for mean/covariance ODEs
dt = 1e-3; T = 15; N = round(T/dt);
mx = [0;0];
P  = zeros(2);

mx_hist = zeros(2,N);
var_hist = zeros(1,N);

for n = 1:N
    mdot = A*mx + B*u_mean;
    Pdot = A*P + P*A' + Gamma*q*Gamma';
    mx = mx + dt*mdot;
    P  = P  + dt*Pdot;

    mx_hist(:,n) = mx;
    var_hist(n) = P(1,1);
end

% Steady-state covariance via Lyapunov equation
% A*Pss + Pss*A' + Gamma*q*Gamma' = 0
Pss = lyap(A, Gamma*q*Gamma');

fprintf('Final propagated mean (theory): [%g, %g]\n', mx(1), mx(2));
fprintf('Final displacement variance (theory): %g\n', P(1,1));
disp('Steady-state covariance Pss (Lyapunov solution):');
disp(Pss);

% Monte Carlo verification (Euler-Maruyama)
M = 3000;
X = zeros(2,M);
sqrt_qdt = sqrt(q*dt);

for n = 1:N
    W = randn(1,M);
    drift = A*X + B*u_mean;
    diff  = Gamma * (sqrt_qdt * W);
    X = X + dt*drift + diff;
end

x1 = X(1,:);
fprintf('Monte Carlo displacement mean: %g\n', mean(x1));
fprintf('Monte Carlo displacement variance: %g\n', var(x1,1));

% -------------------------------------------------------------
% Simulink recipe (programmatic outline)
% -------------------------------------------------------------
% The following commands create a simple Simulink model with:
% Constant mean force + Band-Limited White Noise -> Sum -> State-Space block
%
% Uncomment and run in MATLAB with Simulink installed.
%
% modelName = 'Chapter17_Lesson3_Simulink';
% new_system(modelName); open_system(modelName);
% add_block('simulink/Sources/Constant', [modelName '/MeanForce'], ...
%           'Value', num2str(u_mean), 'Position', [30 40 80 70]);
% add_block('simulink/Sources/Band-Limited White Noise', [modelName '/Noise'], ...
%           'NoisePower', 'q', 'SampleTime', num2str(dt), 'Position', [30 100 160 130]);
% add_block('simulink/Math Operations/Sum', [modelName '/Sum'], ...
%           'Inputs', '++', 'Position', [210 55 240 115]);
% add_block('simulink/Continuous/State-Space', [modelName '/Plant'], ...
%           'A', mat2str(A), 'B', mat2str(B), 'C', mat2str(C), 'D', '0', ...
%           'Position', [300 50 450 120]);
% add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
%           'Position', [500 60 560 110]);
% add_line(modelName, 'MeanForce/1', 'Sum/1');
% add_line(modelName, 'Noise/1', 'Sum/2');
% add_line(modelName, 'Sum/1', 'Plant/1');
% add_line(modelName, 'Plant/1', 'Scope/1');
% set_param(modelName, 'StopTime', num2str(T));
% save_system(modelName);
