% Chapter13_Lesson1.m
% System Dynamics — Chapter 13, Lesson 1
% Modeling Two- and Multi-Degree-of-Freedom Mechanical Systems.
%
% This script:
%   1) Defines M, C, K for a 2-DOF system,
%   2) Converts to first-order state-space,
%   3) Simulates free and forced response using ODE45,
%   4) Shows how to use the State-Space block in Simulink (programmatically).

clear; clc;

%% Parameters
m1 = 1.0; m2 = 0.8;
k1 = 200.0; k2 = 150.0; k3 = 100.0;
c1 = 1.5;   c2 = 1.0;   c3 = 0.8;

%% Matrices (M qdd + C qd + K q = f)
M = diag([m1, m2]);
C = [c1+c2, -c2;
     -c2,   c2+c3];
K = [k1+k2, -k2;
     -k2,   k2+k3];

%% State-space: x = [q; qd], xdot = A x + B f
n = 2;
A = [zeros(n), eye(n);
     -M\K,    -M\C];
B = [zeros(n); M\eye(n)];

%% Free response: f(t)=0
force_free = @(t) [0; 0];

rhs = @(t, x) A*x + B*force_free(t);

tspan = [0 8];
x0 = [0.02; -0.01; 0; 0];

opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t, x] = ode45(rhs, tspan, x0, opts);

q1 = x(:,1); q2 = x(:,2);

figure; plot(t, q1, t, q2); grid on;
xlabel('t [s]'); ylabel('displacement [m]');
legend('q1 free', 'q2 free');

%% Forced response: harmonic force on mass 1
F0 = 5.0; w = 12.0;
force_forced = @(t) [F0*cos(w*t); 0];

rhs2 = @(t, x) A*x + B*force_forced(t);
[t2, x2] = ode45(rhs2, tspan, x0, opts);

figure; plot(t2, x2(:,1), t2, x2(:,2)); grid on;
xlabel('t [s]'); ylabel('displacement [m]');
legend('q1 forced', 'q2 forced');

%% Simulink note (programmatic model creation)
% You can simulate the same state-space system with Simulink's State-Space block.
% The block expects (A,B,C,D). For example, to output q1 and q2:
% Csim = [eye(2), zeros(2)];
% Dsim = zeros(2,2);
%
% To create a minimal Simulink model automatically:
try
    mdl = 'Chapter13_Lesson1_Simulink';
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);

    add_block('simulink/Continuous/State-Space', [mdl '/Plant'], 'Position', [120 90 260 160]);
    set_param([mdl '/Plant'], 'A', 'A', 'B', 'B', 'C', 'Csim', 'D', 'Dsim');

    add_block('simulink/Sources/From Workspace', [mdl '/Force'], 'Position', [20 100 90 130]);
    set_param([mdl '/Force'], 'VariableName', 'force_ts');

    add_block('simulink/Sinks/To Workspace', [mdl '/y'], 'Position', [300 105 360 135]);
    set_param([mdl '/y'], 'VariableName', 'yout', 'SaveFormat', 'Array');

    add_line(mdl, 'Force/1', 'Plant/1');
    add_line(mdl, 'Plant/1', 'y/1');

    % Build a time series force input: [f1(t) f2(t)]
    tgrid = linspace(0, 8, 2001)';
    f1 = F0*cos(w*tgrid);
    f2 = zeros(size(tgrid));
    force_ts = timeseries([f1 f2], tgrid);

    Csim = [eye(2), zeros(2)];
    Dsim = zeros(2,2);

    set_param(mdl, 'StopTime', '8');
    sim(mdl);

    fprintf('Simulink ran. yout contains [q1 q2].\n');
catch ME
    fprintf('Simulink section skipped (reason: %s).\n', ME.message);
end
