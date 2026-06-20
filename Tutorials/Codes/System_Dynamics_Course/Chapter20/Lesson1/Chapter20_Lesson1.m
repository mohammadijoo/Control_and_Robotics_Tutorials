% Chapter20_Lesson1.m
% System Dynamics — Chapter 20, Lesson 1
% Nonlinear Maps and Continuous-Time Chaotic Systems (Logistic Map, Lorenz System)
%
% This script:
% 1) Simulates the logistic map and draws a cobweb plot.
% 2) Simulates the Lorenz system using ode45 and (optionally) a from-scratch RK4.
% 3) Programmatically builds a simple Simulink model for the Lorenz ODE (optional).
%
% Requirements:
% - MATLAB
% - For Simulink section: Simulink installed

clear; close all; clc;

%% 1) Logistic map
r = 3.8;
x0 = 0.2;
N = 80;
x = zeros(N+1,1);
x(1) = x0;
for k = 1:N
    x(k+1) = r*x(k)*(1 - x(k));
end

figure; plot(0:N, x, 'LineWidth', 1.2);
grid on; xlabel('n'); ylabel('x_n');
title(sprintf('Logistic map time series (r=%.3g, x0=%.3g)', r, x0));

% Cobweb plot
f = @(u) r*u.*(1-u);
gridx = linspace(0,1,600);
figure; plot(gridx, f(gridx), 'LineWidth', 1.2); hold on;
plot(gridx, gridx, 'LineWidth', 1.2);
xk = x0;
for k = 1:40
    yk = f(xk);
    plot([xk xk], [xk yk], 'k-');
    plot([xk yk], [yk yk], 'k-');
    xk = yk;
end
xlim([0 1]); ylim([0 1]); grid on;
xlabel('x_n'); ylabel('x_{n+1}');
title(sprintf('Cobweb plot (r=%.3g, x0=%.3g)', r, x0));
legend('f(x)','y=x','Location','best');

%% 2) Lorenz system via ode45
sigma = 10; rho = 28; beta = 8/3;
lorenz = @(t,s) [ sigma*(s(2)-s(1));
                  s(1)*(rho - s(3)) - s(2);
                  s(1)*s(2) - beta*s(3) ];

tspan = [0 40];
s0 = [1; 1; 1];
opts = odeset('RelTol',1e-9,'AbsTol',1e-12,'MaxStep',0.01);
[t,S] = ode45(lorenz, tspan, s0, opts);

figure; plot(t, S(:,1), t, S(:,2), t, S(:,3), 'LineWidth', 1.0);
grid on; xlabel('t'); ylabel('states');
title('Lorenz states vs time (ode45)');
legend('x','y','z');

figure; plot3(S(:,1), S(:,2), S(:,3), 'LineWidth', 0.7);
grid on; xlabel('x'); ylabel('y'); zlabel('z');
title('Lorenz attractor (ode45)');

%% 3) Sensitivity demo
eps = 1e-9;
s0b = s0 + [eps; 0; 0];
[t2,S2] = ode45(lorenz, tspan, s0b, opts);

d = vecnorm((S - S2).', 2).'; % Euclidean norm row-wise
figure; semilogy(t, d + 1e-30, 'LineWidth', 1.0);
grid on; xlabel('t'); ylabel('||delta(t)||');
title(sprintf('Sensitivity in Lorenz system (eps=%.1e)', eps));

%% 4) Optional: programmatically build a Simulink model for Lorenz
% This creates a model with three Integrator blocks and algebraic blocks.
% Uncomment to run.
%
% modelName = 'Chapter20_Lesson1_LorenzSimulink';
% new_system(modelName); open_system(modelName);
% set_param(modelName,'StopTime','40');
% 
% % Add blocks
% add_block('simulink/Continuous/Integrator',[modelName '/Int_x'], 'Position',[120 80 150 110]);
% add_block('simulink/Continuous/Integrator',[modelName '/Int_y'], 'Position',[120 160 150 190]);
% add_block('simulink/Continuous/Integrator',[modelName '/Int_z'], 'Position',[120 240 150 270]);
% 
% % Initial conditions
% set_param([modelName '/Int_x'],'InitialCondition','1');
% set_param([modelName '/Int_y'],'InitialCondition','1');
% set_param([modelName '/Int_z'],'InitialCondition','1');
% 
% % Use MATLAB Function block for RHS
% add_block('simulink/User-Defined Functions/MATLAB Function',[modelName '/LorenzRHS'],...
%     'Position',[250 120 380 240]);
% 
% rhsCode = sprintf([ ...
% 'function dx = f(x)\n' ...
% '%% x = [x;y;z]\n' ...
% 'sigma=10; rho=28; beta=8/3;\n' ...
% 'dx = zeros(3,1);\n' ...
% 'dx(1)=sigma*(x(2)-x(1));\n' ...
% 'dx(2)=x(1)*(rho-x(3))-x(2);\n' ...
% 'dx(3)=x(1)*x(2)-beta*x(3);\n' ...
% 'end\n']);
% set_param([modelName '/LorenzRHS'],'Script',rhsCode);
% 
% % Mux state vector
% add_block('simulink/Signal Routing/Mux',[modelName '/Mux'], 'Inputs','3', 'Position',[190 135 210 215]);
% 
% % Demux derivatives
% add_block('simulink/Signal Routing/Demux',[modelName '/Demux'], 'Outputs','3', 'Position',[410 140 430 210]);
% 
% % Connect integrators to mux
% add_line(modelName,'Int_x/1','Mux/1');
% add_line(modelName,'Int_y/1','Mux/2');
% add_line(modelName,'Int_z/1','Mux/3');
% 
% % Connect mux to RHS to demux
% add_line(modelName,'Mux/1','LorenzRHS/1');
% add_line(modelName,'LorenzRHS/1','Demux/1');
% 
% % Feed derivatives to integrators
% add_line(modelName,'Demux/1','Int_x/1','autorouting','on');
% add_line(modelName,'Demux/2','Int_y/1','autorouting','on');
% add_line(modelName,'Demux/3','Int_z/1','autorouting','on');
% 
% % Add scopes
% add_block('simulink/Sinks/Scope',[modelName '/Scope'], 'Position',[520 130 550 200]);
% add_line(modelName,'Mux/1','Scope/1');
% 
% save_system(modelName);
% disp(['Built model: ' modelName]);
