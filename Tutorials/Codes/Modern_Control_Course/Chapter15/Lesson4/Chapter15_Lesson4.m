% Chapter15_Lesson4.m
% Condition Numbers and Sensitivity in State Reconstruction
%
% Required core functions: expm, eig, cond, mldivide
% Related MATLAB/Simulink Modern Control tools:
%   Control System Toolbox: ss, obsv, gram, lyap
%   Simulink: State-Space block for output simulation

clear; clc;

epsSensor = 0.02;
A = [-1 0; 0 -2];
C = [1 epsSensor];

T = 5;
N = 4000;
t = linspace(0, T, N + 1);
dt = t(2) - t(1);

x0_true = [1; -1.5];

W = zeros(2);
b = zeros(2,1);

for k = 1:N
    tm = 0.5 * (t(k) + t(k + 1));
    Phi = expm(A * tm);
    H = C * Phi;

    y_clean = H * x0_true;
    y_noisy = y_clean + 1.0e-3 * sin(37 * tm);

    W = W + (H' * H) * dt;
    b = b + H' * y_noisy * dt;
end

W = 0.5 * (W + W');
x0_hat = W \ b;

alpha = 1.0e-5;
x0_ridge = (W + alpha * eye(2)) \ b;

lambda = eig(W);
kappa = cond(W);

disp('Finite-horizon observability Gramian W_o(T):');
disp(W);
disp('Eigenvalues:');
disp(lambda.');
fprintf('2-norm condition number: %.12e\n\n', kappa);

disp('True x0:');
disp(x0_true.');
disp('Least-squares estimate:');
disp(x0_hat.');
disp('Ridge estimate:');
disp(x0_ridge.');

% Optional Control System Toolbox checks
if exist('obsv', 'file') == 2
    O = obsv(A, C);
    fprintf('\nRank of Kalman observability matrix: %d\n', rank(O));
end

if exist('ss', 'file') == 2
    sys = ss(A, [], C, []);
    disp('State-space model created with ss(A,[],C,[]).');
end

% Optional Simulink note:
% A corresponding Simulink model can be built with a State-Space block using
% A, B=[], C, D=[] for autonomous response, with initial condition x0_true.
% The reconstruction algorithm above is then applied to the logged output y(t).
