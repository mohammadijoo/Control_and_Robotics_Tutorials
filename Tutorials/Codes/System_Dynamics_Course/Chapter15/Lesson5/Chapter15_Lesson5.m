% Chapter15_Lesson5.m
% Numerical Stability, Error Control, and Model Verification via Simulation
% MATLAB/Simulink implementation for Chapter 15, Lesson 5 (System Dynamics)
%
% Features
%   1) Explicit Euler stability test on y' = lambda y
%   2) ode45 vs ode15s comparison (error-control behavior)
%   3) RK4 convergence verification + observed order
%   4) Simulink solver configuration example (if a model exists)

clear; clc;

%% 1) Absolute-stability demo (explicit Euler)
lambda = -50;
hs = [0.01 0.03 0.05 0.06];
N = 30;
fprintf('=== Explicit Euler stability demo ===\n');
for h = hs
    amp = abs(1 + h*lambda);
    y = 1;
    for n = 1:N
        y = y + h*lambda*y;
    end
    fprintf('h = %.3f, |1+h*lambda| = %.6f, y_N = %.6e\n', h, amp, y);
end
fprintf('\n');

%% 2) Damped oscillator: adaptive solvers and verification target
omega_n = 4.0; zeta = 0.05;
f = @(t,x) [x(2); -2*zeta*omega_n*x(2) - omega_n^2*x(1)];
tspan = [0 12];
x0 = [1; 0];

% ode45 (nonstiff, adaptive)
opts45 = odeset('RelTol',1e-6,'AbsTol',1e-9);
[t45, x45] = ode45(f, tspan, x0, opts45);

% ode15s (stiff-capable, adaptive)
opts15s = odeset('RelTol',1e-6,'AbsTol',1e-9);
[t15s, x15s] = ode15s(f, tspan, x0, opts15s);

qExact45 = exactQ(t45, omega_n, zeta, x0(1), x0(2));
qExact15s = exactQ(t15s, omega_n, zeta, x0(1), x0(2));

err45 = max(abs(x45(:,1) - qExact45));
err15s = max(abs(x15s(:,1) - qExact15s));

E45 = 0.5*(x45(:,2).^2 + omega_n^2*x45(:,1).^2);
energyNonInc45 = all(diff(E45) <= 1e-8);

fprintf('=== Adaptive solver comparison ===\n');
fprintf('ode45 steps = %d, max|q-q_exact| = %.3e, energy nonincreasing = %d\n', ...
    numel(t45)-1, err45, energyNonInc45);
fprintf('ode15s steps = %d, max|q-q_exact| = %.3e\n', numel(t15s)-1, err15s);
fprintf('\n');

%% 3) Fixed-step RK4 convergence verification
fprintf('=== RK4 convergence verification ===\n');
hs = [0.2 0.1 0.05 0.025];
errs = zeros(size(hs));
tf = 5.0;
for i = 1:numel(hs)
    h = hs(i);
    [t, X] = rk4Fixed(f, [0 tf], x0, h);
    qEx = exactQ(t, 4.0, 0.1, x0(1), x0(2));
    errs(i) = max(abs(X(:,1) - qEx));
    fprintf('h = %.4f, max error = %.6e\n', h, errs(i));
end
for i = 1:numel(hs)-1
    pObs = log(errs(i)/errs(i+1))/log(2);
    fprintf('p_obs(%.4f -> %.4f) = %.4f\n', hs(i), hs(i+1), pObs);
end
fprintf('\n');

%% 4) Simulink solver configuration snippet (optional)
% If you have a Simulink model named 'Chapter15Lesson5Model', this block
% configures solver and tolerances programmatically.
mdl = 'Chapter15Lesson5Model';
if exist([mdl '.slx'],'file') || exist([mdl '.mdl'],'file')
    load_system(mdl);
    set_param(mdl, ...
        'SolverType','Variable-step', ...
        'Solver','ode45', ...
        'RelTol','1e-6', ...
        'AbsTol','1e-9', ...
        'MaxStep','0.2');
    fprintf('Configured Simulink model "%s" with variable-step ode45.\n', mdl);

    % For stiff behavior, you may switch to ode15s:
    % set_param(mdl, 'Solver', 'ode15s');

    % To simulate:
    % simOut = sim(mdl, 'StopTime', '12');
else
    fprintf('No Simulink model file found. Skipping set_param demo.\n');
end

%% Local functions
function [t, X] = rk4Fixed(f, tspan, x0, h)
    t0 = tspan(1); tf = tspan(2);
    N = round((tf - t0)/h);
    t = linspace(t0, tf, N+1)';
    X = zeros(N+1, numel(x0));
    X(1,:) = x0(:).';
    for k = 1:N
        tk = t(k);
        xk = X(k,:).';
        k1 = f(tk, xk);
        k2 = f(tk + h/2, xk + h*k1/2);
        k3 = f(tk + h/2, xk + h*k2/2);
        k4 = f(tk + h,   xk + h*k3);
        X(k+1,:) = (xk + h*(k1 + 2*k2 + 2*k3 + k4)/6).';
    end
end

function q = exactQ(t, omega_n, zeta, q0, v0)
    wd = omega_n * sqrt(1 - zeta^2);
    A = q0;
    B = (v0 + zeta*omega_n*q0)/wd;
    q = exp(-zeta*omega_n*t) .* (A*cos(wd*t) + B*sin(wd*t));
end
