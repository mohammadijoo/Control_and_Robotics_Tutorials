% Chapter 3 - Lesson 2: Curvature and Turning Radius Limits (MATLAB/Simulink)
%
% This script:
%   1) Computes discrete curvature along a sampled 2D path.
%   2) Converts curvature to bicycle steering angle delta = atan(L*kappa).
%   3) Checks steering limits and lateral-acceleration speed limits v_max(s).
%   4) (Optional) Builds a minimal Simulink bicycle-kinematics model programmatically.
%
% Recommended toolboxes (optional):
%   - Robotics System Toolbox (for integration with mobile robot workflows)
%   - Simulink (for model generation)
%
% Author: Course generator

clear; clc;

%% Parameters
L = 0.35;                      % wheelbase [m]
delta_max = deg2rad(28);        % steering angle limit [rad]
a_lat_max = 2.0;                % lateral acceleration limit [m/s^2]

%% Example path (students may replace)
N = 401;
s = linspace(0, 12, N);
x = s;
y = 1.2*sin(0.7*s) + 0.2*sin(2.2*s);
P = [x(:), y(:)];

%% Discrete curvature using 3-point circumcircle formula
kappa = zeros(N,1);
for i = 2:N-1
    p0 = P(i-1,:);
    p1 = P(i,:);
    p2 = P(i+1,:);

    v01 = p1 - p0;
    v02 = p2 - p0;
    v12 = p2 - p1;

    a = norm(v12);
    b = norm(v02);
    c = norm(v01);

    area2 = v01(1)*v02(2) - v01(2)*v02(1); % signed double area

    denom = a*b*c;
    if denom < 1e-12
        kappa(i) = 0;
    else
        kappa(i) = 2*area2/denom;
    end
end

%% Bicycle steering mapping and limits
delta = atan(L*kappa);
kappa_max_steer = tan(delta_max)/L;

steer_ok = abs(delta) <= delta_max + 1e-12;
fprintf('kappa_max_steer = %.4f 1/m\n', kappa_max_steer);
fprintf('Peak |kappa|    = %.4f 1/m\n', max(abs(kappa)));
fprintf('Steering-feasible? %s\n', string(all(abs(kappa) <= kappa_max_steer + 1e-12)));

%% Lateral acceleration speed limits: v <= sqrt(a_lat_max/|kappa|)
vmax = inf(N,1);
mask = abs(kappa) > 1e-12;
vmax(mask) = sqrt(a_lat_max ./ abs(kappa(mask)));

%% Visualizations
figure; plot(x,y,'LineWidth',1.5); axis equal; grid on;
title('Path (x,y)'); xlabel('x [m]'); ylabel('y [m]');

figure; plot(s,kappa,'LineWidth',1.5); grid on;
title('Curvature kappa(s)'); xlabel('s (index)'); ylabel('kappa [1/m]');

figure; plot(s,vmax,'LineWidth',1.5); grid on;
title('Speed upper bound from lateral acceleration'); xlabel('s (index)'); ylabel('v_{max} [m/s]');

%% Optional: Build a simple Simulink bicycle kinematics model
% This creates a model with states (x,y,theta) and inputs (v, delta).
% To run it:
%   - Ensure Simulink is installed.
%   - Uncomment the lines below.
%
% modelName = 'Chapter3_Lesson2_BicycleModel';
% buildSimulinkBicycleModel(modelName, L);
% open_system(modelName);

function buildSimulinkBicycleModel(modelName, L)
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    % Blocks
    add_block('simulink/Sources/In1', [modelName '/v']);
    add_block('simulink/Sources/In1', [modelName '/delta']);
    add_block('simulink/Math Operations/Trigonometric Function', [modelName '/cos'], 'Operator', 'cos');
    add_block('simulink/Math Operations/Trigonometric Function', [modelName '/sin'], 'Operator', 'sin');
    add_block('simulink/Math Operations/Trigonometric Function', [modelName '/tan'], 'Operator', 'tan');

    add_block('simulink/Math Operations/Product', [modelName '/vcos']);
    add_block('simulink/Math Operations/Product', [modelName '/vsin']);
    add_block('simulink/Math Operations/Product', [modelName '/v_tan']);
    add_block('simulink/Math Operations/Gain', [modelName '/1_over_L'], 'Gain', num2str(1/L));

    add_block('simulink/Continuous/Integrator', [modelName '/Int_x']);
    add_block('simulink/Continuous/Integrator', [modelName '/Int_y']);
    add_block('simulink/Continuous/Integrator', [modelName '/Int_theta']);

    add_block('simulink/Sinks/Out1', [modelName '/x']);
    add_block('simulink/Sinks/Out1', [modelName '/y']);
    add_block('simulink/Sinks/Out1', [modelName '/theta']);

    % Connections
    add_line(modelName, 'delta/1', 'tan/1');
    add_line(modelName, 'tan/1', 'v_tan/2');
    add_line(modelName, 'v/1', 'v_tan/1');
    add_line(modelName, 'v_tan/1', '1_over_L/1');
    add_line(modelName, '1_over_L/1', 'Int_theta/1');
    add_line(modelName, 'Int_theta/1', 'cos/1');
    add_line(modelName, 'Int_theta/1', 'sin/1');
    add_line(modelName, 'v/1', 'vcos/1');
    add_line(modelName, 'cos/1', 'vcos/2');
    add_line(modelName, 'v/1', 'vsin/1');
    add_line(modelName, 'sin/1', 'vsin/2');
    add_line(modelName, 'vcos/1', 'Int_x/1');
    add_line(modelName, 'vsin/1', 'Int_y/1');

    add_line(modelName, 'Int_x/1', 'x/1');
    add_line(modelName, 'Int_y/1', 'y/1');
    add_line(modelName, 'Int_theta/1', 'theta/1');

    set_param(modelName, 'StopTime', '10');
    save_system(modelName);
end
