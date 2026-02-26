% Chapter18_Lesson5.m
% Case Study: Agricultural / Delivery Robots
% MATLAB + Simulink-oriented script for risk-aware mission calculations

clear; clc;

%% 1) Agricultural coverage and energy model
fieldLength = 120;   % m
fieldWidth  = 48;    % m
toolWidth   = 2.4;   % m
overlap     = 0.12;  % 12 percent
v_ag        = 1.4;   % m/s
turnLoss    = 7.0;   % s per turn

wEff   = toolWidth * (1 - overlap);
nPass  = ceil(fieldWidth / wEff);
distAg = nPass * fieldLength;
tAg    = distAg / v_ag + (nPass - 1) * turnLoss;

mAg    = 180;      % kg
crr    = 0.03;
rhoAir = 1.2;
CdA    = 0.45;
Paux   = 80;
slope  = 0.03;     % rad
g      = 9.81;

Froll = crr * mAg * g * cos(slope);
Fgrade = mAg * g * sin(slope);
Fdrag = 0.5 * rhoAir * CdA * v_ag^2;
Pag = max(0, (Froll + Fgrade + Fdrag) * v_ag) + Paux;
Eag = Pag * tAg;

fprintf('Agricultural passes = %d\n', nPass);
fprintf('Agricultural mission time = %.2f min\n', tAg / 60);
fprintf('Agricultural mission energy = %.2f kJ\n', Eag / 1000);

%% 2) Delivery ETA chance constraint
muSeg = [110, 95, 150];      % s
sigSeg = [12, 10, 20];       % s
deadline = 390;              % s
z95 = 1.645;

muTot = sum(muSeg);
sigTot = sqrt(sum(sigSeg.^2));
etaBound = muTot + z95 * sigTot;
feasibleDeadline = etaBound <= deadline;

fprintf('Delivery ETA bound (95%%) = %.2f s\n', etaBound);
fprintf('Delivery deadline feasible = %d\n', feasibleDeadline);

%% 3) Safety speed bound from stopping distance
clearance = 4.0;  % m
tReaction = 0.5;  % s
mu = 0.6;
margin = 0.6;

% Solve v*tReaction + v^2/(2*mu*g) + margin <= clearance
a = 1 / (2 * mu * g);
b = tReaction;
c = -(clearance - margin);
vmax = (-b + sqrt(b^2 - 4*a*c)) / (2*a);
fprintf('Safe speed from clearance = %.3f m/s\n', vmax);

%% 4) Simulink model (programmatic) for longitudinal speed tracking
% This builds a simple model: reference -> PI -> saturation -> vehicle integrator
mdl = 'Chapter18_Lesson5_Simulink';
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
if exist([mdl '.slx'], 'file')
    delete([mdl '.slx']);
end

new_system(mdl);
open_system(mdl);

add_block('simulink/Sources/Step', [mdl '/v_ref'], 'Position', [30 40 60 70]);
set_param([mdl '/v_ref'], 'Time', '0', 'Before', '0', 'After', num2str(vmax));

add_block('simulink/Math Operations/Sum', [mdl '/sum'], 'Inputs', '+-', ...
    'Position', [110 35 130 75]);

add_block('simulink/Continuous/PID Controller', [mdl '/PI'], ...
    'P', '2.0', 'I', '1.0', 'D', '0.0', 'Position', [170 25 240 85]);

add_block('simulink/Discontinuities/Saturation', [mdl '/sat_acc'], ...
    'UpperLimit', '1.2', 'LowerLimit', '-1.8', 'Position', [280 35 330 75]);

add_block('simulink/Continuous/Transfer Fcn', [mdl '/vehicle'], ...
    'Numerator', '[1]', 'Denominator', '[0.6 1]', ...
    'Position', [380 35 460 75]);

add_block('simulink/Sinks/Scope', [mdl '/scope'], 'Position', [520 35 550 75]);

add_block('simulink/Signal Routing/Mux', [mdl '/mux'], ...
    'Inputs', '2', 'Position', [490 95 510 135]);

add_line(mdl, 'v_ref/1', 'sum/1');
add_line(mdl, 'sum/1', 'PI/1');
add_line(mdl, 'PI/1', 'sat_acc/1');
add_line(mdl, 'sat_acc/1', 'vehicle/1');
add_line(mdl, 'vehicle/1', 'sum/2');
add_line(mdl, 'vehicle/1', 'mux/1');
add_line(mdl, 'v_ref/1', 'mux/2');
add_line(mdl, 'mux/1', 'scope/1');

set_param(mdl, 'StopTime', '8');
save_system(mdl);

% Uncomment to simulate:
% simOut = sim(mdl);
% disp('Simulink model generated and saved.');
