% Chapter7_Lesson5.m
% Modern Control — Chapter 7, Lesson 5: Numerical Simulation of State Equations
%
% Demonstrates:
%  1) Continuous-time simulation using ode45 (adaptive RK)
%  2) Fixed-step RK4 (implemented explicitly)
%  3) Exact ZOH discretization: Ad = expm(Ah), Bd = integral_0^h expm(A tau) B d tau
%     computed via Van Loan augmented exponential.
%
% Run:
%   Chapter7_Lesson5

clear; clc;

A = [0 1; -2 -3];
B = [0; 1];
x0 = [1; 0];

u = @(t) sin(2*t);
f = @(t,x) A*x + B*u(t);

t0 = 0; tf = 10; h = 0.01;
tgrid = (t0:h:tf)';

%% 1) ode45 reference
opts = odeset('RelTol',1e-9,'AbsTol',1e-12);
[ts_ref, xs_ref] = ode45(f, [t0 tf], x0, opts);
x_ref = interp1(ts_ref, xs_ref, tgrid);

%% 2) Fixed-step RK4
N = numel(tgrid);
x_rk4 = zeros(N, numel(x0));
x_rk4(1,:) = x0.';
x = x0;
for k = 1:N-1
    t = tgrid(k);
    k1 = f(t, x);
    k2 = f(t + 0.5*h, x + 0.5*h*k1);
    k3 = f(t + 0.5*h, x + 0.5*h*k2);
    k4 = f(t + h, x + h*k3);
    x = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
    x_rk4(k+1,:) = x.';
end

%% 3) Exact ZOH discretization (Van Loan)
[Ad, Bd] = van_loan_discretization(A, B, h);

x_zoh = zeros(N, numel(x0));
x_zoh(1,:) = x0.';
x = x0;
for k = 1:N-1
    t = tgrid(k);
    uk = u(t); % ZOH sample
    x = Ad*x + Bd*uk;
    x_zoh(k+1,:) = x.';
end

%% Error comparison
err_rk4 = vecnorm(x_rk4 - x_ref, 2, 2);
err_zoh = vecnorm(x_zoh - x_ref, 2, 2);

fprintf('Max error RK4 vs ode45: %.3e\n', max(err_rk4));
fprintf('Max error ZOH vs ode45: %.3e\n', max(err_zoh));

%% Optional: Simulink programmatic model (no images)
% This creates a simple model using a State-Space block to simulate:
%   xdot = A x + B u ; y = x
try
    mdl = 'Chapter7_Lesson5_Simulink';
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);

    add_block('simulink/Sources/Sine Wave', [mdl '/u']);
    set_param([mdl '/u'], 'Amplitude', '1', 'Frequency', '2'); % sin(2 t)

    add_block('simulink/Continuous/State-Space', [mdl '/SS']);
    set_param([mdl '/SS'], 'A', mat2str(A), 'B', mat2str(B), 'C', mat2str(eye(2)), 'D', 'zeros(2,1)');

    add_block('simulink/Sinks/To Workspace', [mdl '/x_out']);
    set_param([mdl '/x_out'], 'VariableName', 'x_sim', 'SaveFormat', 'Array');

    add_line(mdl, 'u/1', 'SS/1');
    add_line(mdl, 'SS/1', 'x_out/1');

    set_param(mdl, 'StopTime', num2str(tf));
    sim(mdl);
    % x_sim is available in workspace
    disp('Simulink model created and simulated: Chapter7_Lesson5_Simulink');
catch ME
    disp('Simulink step skipped (Simulink not available or error):');
    disp(ME.message);
end

%% ---- Local function: Van Loan discretization ----
function [Ad, Bd] = van_loan_discretization(A, B, h)
    % expm([A B; 0 0] h) = [Ad Bd; 0 I]
    n = size(A,1);
    m = size(B,2);
    M = zeros(n+m, n+m);
    M(1:n,1:n) = A;
    M(1:n,n+1:n+m) = B;
    E = expm(M*h);
    Ad = E(1:n,1:n);
    Bd = E(1:n,n+1:n+m);
end
