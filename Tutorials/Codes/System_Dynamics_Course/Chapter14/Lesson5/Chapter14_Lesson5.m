% Chapter14_Lesson5.m
% Limit Cycles, Multiple Equilibria, and Basic Bifurcation Notions
% MATLAB + basic Simulink automation (if Simulink is installed)

clear; clc; close all;

%% Part A: Van der Pol oscillator (ode45) and limit-cycle period estimation
mu = 1.0;
f = @(t,z) [z(2); mu*(1 - z(1)^2)*z(2) - z(1)];
tspan = [0 80];
z0 = [2; 0.1];
opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t,z] = ode45(f, tspan, z0, opts);

% phase portrait (transient removed)
idx0 = floor(numel(t)/2);
figure('Name','Van der Pol Phase Portrait');
plot(z(idx0:end,1), z(idx0:end,2), 'LineWidth', 1.5); grid on;
xlabel('x'); ylabel('x_dot');
title('Van der Pol Limit Cycle (mu = 1)');

% Poincare crossing estimate: y = 0 upward, x > 0
crossings = [];
for k = idx0:(numel(t)-1)
    if z(k,2) < 0 && z(k+1,2) >= 0
        alpha = -z(k,2)/(z(k+1,2)-z(k,2) + eps);
        tc = t(k) + alpha*(t(k+1)-t(k));
        xc = z(k,1) + alpha*(z(k+1,1)-z(k,1));
        if xc > 0
            crossings(end+1) = tc; %#ok<SAGROW>
        end
    end
end
if numel(crossings) >= 3
    P = mean(diff(crossings(max(1,end-4):end)));
else
    P = NaN;
end
A = max(abs(z(idx0:end,1)));
fprintf('Van der Pol limit-cycle estimate: amplitude = %.6f, period = %.6f\n', A, P);

%% Part B: Basic 1D bifurcation normal forms
r = linspace(-2, 2, 801);
pf_stable = []; pf_unstable = [];
sn_stable = []; sn_unstable = [];

for rv = r
    % Pitchfork: x*=0 always
    lam0 = rv;
    if lam0 < 0
        pf_stable = [pf_stable; rv 0]; %#ok<AGROW>
    else
        pf_unstable = [pf_unstable; rv 0]; %#ok<AGROW>
    end
    if rv >= 0
        xe = [sqrt(rv), -sqrt(rv)];
        for xeq = xe
            lam = rv - 3*xeq^2;
            if lam < 0
                pf_stable = [pf_stable; rv xeq]; %#ok<AGROW>
            else
                pf_unstable = [pf_unstable; rv xeq]; %#ok<AGROW>
            end
        end
    end

    % Saddle-node: x*= +-sqrt(r) for r>=0
    if rv >= 0
        xplus = sqrt(rv); xminus = -sqrt(rv);
        if -2*xplus < 0, sn_stable = [sn_stable; rv xplus]; else, sn_unstable = [sn_unstable; rv xplus]; end %#ok<AGROW>
        if -2*xminus < 0, sn_stable = [sn_stable; rv xminus]; else, sn_unstable = [sn_unstable; rv xminus]; end %#ok<AGROW>
    end
end

figure('Name','Bifurcation Branches');
hold on; grid on;
if ~isempty(pf_stable),   plot(pf_stable(:,1), pf_stable(:,2),  'LineWidth', 1.6); end
if ~isempty(pf_unstable), plot(pf_unstable(:,1), pf_unstable(:,2),'--','LineWidth', 1.2); end
if ~isempty(sn_stable),   plot(sn_stable(:,1), sn_stable(:,2),   'LineWidth', 1.6); end
if ~isempty(sn_unstable), plot(sn_unstable(:,1), sn_unstable(:,2),'--','LineWidth', 1.2); end
xlabel('r'); ylabel('x^*');
title('Pitchfork and Saddle-Node Equilibrium Branches');
legend('Pitchfork stable','Pitchfork unstable','Saddle-node stable','Saddle-node unstable');

%% Part C: Simulink (optional) - programmatically build a Van der Pol model
% This section creates a simple Simulink model with two integrators if Simulink exists.
if license('test','Simulink') && exist('new_system','file')
    mdl = 'Chapter14_Lesson5_SimulinkModel';
    if bdIsLoaded(mdl), close_system(mdl,0); end
    if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end
    new_system(mdl); open_system(mdl);

    add_block('simulink/Sources/Constant', [mdl '/mu'], 'Position', [30 40 60 60], 'Value', '1');
    add_block('simulink/Continuous/Integrator', [mdl '/Int_x'], 'Position', [420 90 450 120]);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_y'], 'Position', [420 180 450 210]);

    add_block('simulink/Math Operations/Product', [mdl '/x2'], 'Position', [120 140 150 170], 'Inputs', '**');
    add_block('simulink/Math Operations/Sum', [mdl '/1_minus_x2'], 'Position', [180 140 205 170], 'Inputs', '+-');
    add_block('simulink/Sources/Constant', [mdl '/one'], 'Position', [120 90 150 110], 'Value', '1');

    add_block('simulink/Math Operations/Product', [mdl '/mu_term'], 'Position', [240 120 270 150], 'Inputs', '**');
    add_block('simulink/Math Operations/Product', [mdl '/mu_term_times_y'], 'Position', [300 120 330 150], 'Inputs', '**');
    add_block('simulink/Math Operations/Gain', [mdl '/minus_x'], 'Position', [300 200 340 230], 'Gain', '-1');
    add_block('simulink/Math Operations/Sum', [mdl '/ydot_sum'], 'Position', [360 155 385 185], 'Inputs', '++');

    add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', [520 90 550 120]);

    % Connect x and y states
    add_line(mdl, 'Int_x/1', 'x2/1');
    add_line(mdl, 'Int_x/1', 'x2/2');
    add_line(mdl, 'one/1', '1_minus_x2/1');
    add_line(mdl, 'x2/1', '1_minus_x2/2');
    add_line(mdl, 'mu/1', 'mu_term/1');
    add_line(mdl, '1_minus_x2/1', 'mu_term/2');
    add_line(mdl, 'mu_term/1', 'mu_term_times_y/1');
    add_line(mdl, 'Int_y/1', 'mu_term_times_y/2');
    add_line(mdl, 'mu_term_times_y/1', 'ydot_sum/1');
    add_line(mdl, 'Int_x/1', 'minus_x/1');
    add_line(mdl, 'minus_x/1', 'ydot_sum/2');

    % State equations: xdot = y, ydot = mu(1-x^2)y - x
    add_line(mdl, 'Int_y/1', 'Int_x/1', 'autorouting', 'on');
    add_line(mdl, 'ydot_sum/1', 'Int_y/1', 'autorouting', 'on');
    add_line(mdl, 'Int_x/1', 'Scope/1', 'autorouting', 'on');

    set_param(mdl, 'StopTime', '40');
    save_system(mdl);
    fprintf('Simulink model created: %s.slx\n', mdl);
else
    fprintf('Simulink not available. MATLAB ODE implementation executed.\n');
end
