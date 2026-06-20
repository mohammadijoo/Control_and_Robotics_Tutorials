% Chapter20_Lesson2.m
% System Dynamics — Chapter 20 (Chaos, Complex Dynamics, and Computational Tools)
% Lesson 2: Bifurcation Diagrams, Period Doubling, and Strange Attractors
%
% Outputs:
%   - logistic_bifurcation_matlab.png
%   - lorenz_attractor_matlab.png
%
% Notes:
%   (1) The bifurcation diagram is built by sweeping r, discarding transients,
%       and plotting the last iterates for each r.
%   (2) The Lorenz trajectory is integrated with ode45 (continuous-time ODE).
%   (3) A minimal Simulink model can be generated programmatically (optional).

clear; clc; close all;

%% Part A — Logistic map bifurcation diagram
rmin = 2.5; rmax = 4.0; Nr = 6000;
nTransient = 1200; nKeep = 200;
rs = linspace(rmin, rmax, Nr);
x = 0.123456 * ones(1, Nr);

for k = 1:nTransient
    x = rs .* x .* (1 - x);
end

Rplot = zeros(1, Nr*nKeep);
Xplot = zeros(1, Nr*nKeep);
idx = 1;
for k = 1:nKeep
    x = rs .* x .* (1 - x);
    Rplot(idx:idx+Nr-1) = rs;
    Xplot(idx:idx+Nr-1) = x;
    idx = idx + Nr;
end

figure('Position',[100 100 900 450]);
plot(Rplot, Xplot, '.', 'MarkerSize', 1);
xlabel('r'); ylabel('x (asymptotic samples)');
title('Logistic map bifurcation diagram');
grid on;
saveas(gcf, 'logistic_bifurcation_matlab.png');

%% Part B — Lorenz attractor (ODE) with ode45
sigma = 10; rho = 28; beta = 8/3;
f = @(t, y) [ sigma*(y(2)-y(1));
              y(1)*(rho - y(3)) - y(2);
              y(1)*y(2) - beta*y(3) ];

tspan = [0 40];
y0 = [1; 1; 1];
opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t, Y] = ode45(f, tspan, y0, opts);

% discard transient
mask = t >= 5;
Y2 = Y(mask, :);

figure('Position',[120 120 700 600]);
plot3(Y2(:,1), Y2(:,2), Y2(:,3), 'LineWidth', 0.5);
xlabel('x'); ylabel('y'); zlabel('z');
title('Lorenz attractor trajectory (ode45)');
grid on; view(45, 25);
saveas(gcf, 'lorenz_attractor_matlab.png');

%% Part C — Optional: programmatically build a simple Simulink map model
% This creates a discrete-time logistic map with a Unit Delay and MATLAB Function.
% Requires Simulink installed. If not available, this section will safely skip.
try
    hasSimulink = license('test','Simulink');
catch
    hasSimulink = false;
end

if hasSimulink
    mdl = 'Chapter20_Lesson2_LogisticMap_Simulink';
    new_system(mdl); open_system(mdl);

    add_block('simulink/Sources/Constant', [mdl '/r'], 'Value', '3.7', ...
              'Position', [50 80 90 110]);
    add_block('simulink/Sources/Constant', [mdl '/x0'], 'Value', '0.2', ...
              'Position', [50 160 90 190]);

    add_block('simulink/Discrete/Unit Delay', [mdl '/UnitDelay'], ...
              'InitialCondition', '0.2', 'SampleTime', '1', ...
              'Position', [170 150 220 200]);

    add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/LogisticFcn'], ...
              'Position', [280 130 420 220]);

    % Set MATLAB Function code
    fcn = [ ...
        'function x_next = fcn(x, r)\n' ...
        '%#codegen\n' ...
        'x_next = r * x * (1 - x);\n' ...
        'end\n' ];
    set_param([mdl '/LogisticFcn'], 'Script', fcn);

    add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
              'Position', [480 150 520 190]);

    % Wire: x0 -> UnitDelay input (sets IC only; still connect for convenience)
    add_line(mdl, 'x0/1', 'UnitDelay/1');
    add_line(mdl, 'UnitDelay/1', 'LogisticFcn/1');
    add_line(mdl, 'r/1', 'LogisticFcn/2');
    add_line(mdl, 'LogisticFcn/1', 'UnitDelay/1', 'autorouting', 'on');
    add_line(mdl, 'UnitDelay/1', 'Scope/1');

    set_param(mdl, 'StopTime', '200', 'Solver', 'FixedStepDiscrete');
    save_system(mdl);
    % run with: sim(mdl)
end

disp('Done.');
