% Chapter 14 - Nonlinear System Dynamics
% Lesson 3 - Linearization vs. True Nonlinear Behavior: When Linear Models Fail
%
% File: Chapter14_Lesson3.m
%
% This script demonstrates Example B (2D) and Example C (1D):
%   Example B:
%     xdot = -y + x (x^2 + y^2)
%     ydot =  x + y (x^2 + y^2)
%     Linearization at origin: zdot = A z, A = [0 -1; 1 0]
%   Example C:
%     xdot = -x + x^3 (0 locally stable, but not globally)
%
% Optional: programmatically generate a Simulink model for Example B.

clear; clc;

%% Example B: Compare nonlinear vs linearized
A = [0 -1; 1 0];

f_nl = @(t,z) [ -z(2) + z(1)*(z(1)^2 + z(2)^2);
                 z(1) + z(2)*(z(1)^2 + z(2)^2) ];

f_lin = @(t,z) A*z;

tspan = [0 25];
z0 = [0.2; 0.0];

opts = odeset('RelTol',1e-9,'AbsTol',1e-11);

[t_nl, z_nl] = ode45(f_nl, tspan, z0, opts);
[t_li, z_li] = ode45(f_lin, tspan, z0, opts);

r_nl = sqrt(z_nl(:,1).^2 + z_nl(:,2).^2);
r_li = sqrt(z_li(:,1).^2 + z_li(:,2).^2);

fprintf('Example B: r(0)=%.4f, r(T) nonlinear=%.4f, r(T) linear=%.4f\n', r_nl(1), r_nl(end), r_li(end));

figure; plot(z_li(:,1), z_li(:,2), 'LineWidth', 1.5); hold on;
plot(z_nl(:,1), z_nl(:,2), 'LineWidth', 1.5);
axis equal; grid on; xlabel('x'); ylabel('y');
title('Example B: Phase portrait (linear vs nonlinear)');
legend('Linearized','Nonlinear');
saveas(gcf, 'Chapter14_Lesson3_ExampleB_Phase_matlab.png');

figure; plot(t_li, r_li, 'LineWidth', 1.5); hold on;
plot(t_nl, r_nl, 'LineWidth', 1.5);
grid on; xlabel('t'); ylabel('r');
title('Example B: Radius growth (linear vs nonlinear)');
legend('r(t) linear','r(t) nonlinear');
saveas(gcf, 'Chapter14_Lesson3_ExampleB_Radius_matlab.png');

T = min(length(t_li), length(t_nl));
csvwrite('Chapter14_Lesson3_ExampleB_matlab.csv', [t_li(1:T), z_li(1:T,:), z_nl(1:T,:)]);

%% Example C: Region of attraction issue
fC = @(t,x) -x + x.^3;
fC_lin = @(t,dx) -dx;

x0s = [0.2, 0.9, 1.1, 1.5];
for k = 1:length(x0s)
    x0 = x0s(k);
    [tC, xC] = ode45(fC, [0 10], x0, opts);
    [tL, xL] = ode45(fC_lin, [0 10], x0, opts);
    fprintf('Example C: x0=%.2f -> x(T) nonlinear=%.4f | linear=%.4f\n', x0, xC(end), xL(end));
end

%% Optional Simulink model generation
buildSimulink = false;   % set true if you want to generate a Simulink model
if buildSimulink
    mdl = 'Chapter14_Lesson3_Simulink_ExampleB';
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    new_system(mdl); open_system(mdl);

    % Integrators for x and y
    add_block('simulink/Continuous/Integrator', [mdl '/Int_x'], 'Position', [140 90 170 120]);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_y'], 'Position', [140 170 170 200]);

    % Blocks to compute r2 = x^2 + y^2
    add_block('simulink/Math Operations/Product', [mdl '/x_sq'], 'Position', [260 90 290 120]);
    add_block('simulink/Math Operations/Product', [mdl '/y_sq'], 'Position', [260 170 290 200]);
    add_block('simulink/Math Operations/Sum', [mdl '/sum_r2'], 'Inputs', '++', 'Position', [330 130 360 160]);

    % Products for x*r2 and y*r2
    add_block('simulink/Math Operations/Product', [mdl '/x_r2'], 'Position', [410 90 440 120]);
    add_block('simulink/Math Operations/Product', [mdl '/y_r2'], 'Position', [410 170 440 200]);

    % Sums for xdot = -y + x*r2 and ydot = x + y*r2
    add_block('simulink/Math Operations/Gain', [mdl '/neg_y'], 'Gain', '-1', 'Position', [500 170 530 200]);
    add_block('simulink/Math Operations/Sum', [mdl '/sum_xdot'], 'Inputs', '++', 'Position', [560 110 590 140]);
    add_block('simulink/Math Operations/Sum', [mdl '/sum_ydot'], 'Inputs', '++', 'Position', [560 190 590 220]);

    % Scope
    add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', [700 120 730 190]);

    % Wiring
    add_line(mdl, 'Int_x/1', 'x_sq/1'); add_line(mdl, 'Int_x/1', 'x_sq/2');
    add_line(mdl, 'Int_y/1', 'y_sq/1'); add_line(mdl, 'Int_y/1', 'y_sq/2');
    add_line(mdl, 'x_sq/1', 'sum_r2/1'); add_line(mdl, 'y_sq/1', 'sum_r2/2');

    add_line(mdl, 'Int_x/1', 'x_r2/1'); add_line(mdl, 'sum_r2/1', 'x_r2/2');
    add_line(mdl, 'Int_y/1', 'y_r2/1'); add_line(mdl, 'sum_r2/1', 'y_r2/2');

    add_line(mdl, 'Int_y/1', 'neg_y/1');
    add_line(mdl, 'neg_y/1', 'sum_xdot/1');
    add_line(mdl, 'x_r2/1', 'sum_xdot/2');

    add_line(mdl, 'Int_x/1', 'sum_ydot/1');
    add_line(mdl, 'y_r2/1', 'sum_ydot/2');

    add_line(mdl, 'sum_xdot/1', 'Int_x/1');
    add_line(mdl, 'sum_ydot/1', 'Int_y/1');

    add_line(mdl, 'Int_x/1', 'Scope/1');
    add_line(mdl, 'Int_y/1', 'Scope/2');

    set_param(mdl, 'StopTime', '25', 'Solver', 'ode45');
    save_system(mdl);
    fprintf('Saved Simulink model: %s.slx\n', mdl);
end
