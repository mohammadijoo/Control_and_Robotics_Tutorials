% Chapter6_Lesson3_Simulink.m
% Autonomous Mobile Robots (Control Engineering) - Chapter 6, Lesson 3
% Programmatically builds a Simulink model for a Bayes-filter loop (discrete grid).
%
% Usage (MATLAB + Simulink required):
%   run('Chapter6_Lesson3_Simulink.m')
%
% Output:
%   Chapter6_Lesson3_BayesFilter.slx

clear; clc;

modelName = 'Chapter6_Lesson3_BayesFilter';
new_system(modelName);
open_system(modelName);

% Layout coordinates
x0 = 30; y0 = 60; dx = 170;

% Add blocks
add_block('simulink/Discrete/Unit Delay', [modelName '/BeliefDelay'], ...
    'Position', [x0 y0 x0+70 y0+40], 'InitialCondition', 'ones(121,1)/121');

add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/MotionPredict'], ...
    'Position', [x0+dx y0 x0+dx+140 y0+70]);

add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/SensorUpdate'], ...
    'Position', [x0+2*dx y0 x0+2*dx+140 y0+70]);

add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
    'Position', [x0+3*dx y0 x0+3*dx+60 y0+40]);

% Connect lines
add_line(modelName, 'BeliefDelay/1', 'MotionPredict/1');
add_line(modelName, 'MotionPredict/1', 'SensorUpdate/1');
add_line(modelName, 'SensorUpdate/1', 'BeliefDelay/1', 'autorouting', 'on');
add_line(modelName, 'SensorUpdate/1', 'Scope/1');

% MotionPredict code
mp = find(slroot, '-isa', 'Stateflow.EMChart', 'Path', [modelName '/MotionPredict']);
mp.Script = [
"function bel_bar = MotionPredict(bel)
" + ...
"% Parameters (edit as needed)
" + ...
"N = 121; dx = 0.1; u = 0.5; sigma_u = 0.25;
" + ...
"xs = (0:N-1)' * dx;
" + ...
"bel_bar = zeros(N,1);
" + ...
"for i = 1:N
" + ...
"    mu = xs(i) + u;
" + ...
"    kernel = exp(-0.5*((xs - mu)/sigma_u).^2) / (sqrt(2*pi)*sigma_u);
" + ...
"    bel_bar = bel_bar + bel(i) * kernel;
" + ...
"end
" + ...
"bel_bar = bel_bar / sum(bel_bar);
" + ...
"end
"
];

% SensorUpdate code
su = find(slroot, '-isa', 'Stateflow.EMChart', 'Path', [modelName '/SensorUpdate']);
su.Script = [
"function bel_next = SensorUpdate(bel_bar)
" + ...
"% Parameters (edit as needed)
" + ...
"N = 121; dx = 0.1; beacon_x = 6.0; z = 5.5; sigma_z = 0.35;
" + ...
"xs = (0:N-1)' * dx;
" + ...
"expected = abs(xs - beacon_x);
" + ...
"likelihood = exp(-0.5*((z - expected)/sigma_z).^2) / (sqrt(2*pi)*sigma_z);
" + ...
"bel_next = bel_bar .* likelihood;
" + ...
"bel_next = bel_next / sum(bel_next);
" + ...
"end
"
];

set_param(modelName, 'StopTime', '10', 'Solver', 'FixedStepDiscrete');
save_system(modelName);
fprintf('Saved Simulink model: %s.slx\n', modelName);
