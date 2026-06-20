% Chapter16_Lesson5_Ex1.m
% Programmatically builds a simple Simulink model:
% Step -> Discrete Transfer Fcn -> Scope
% This is useful for sampled-data visualization in the lesson.

modelName = 'Chapter16_Lesson5_Simulink';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

% Add blocks
add_block('simulink/Sources/Step', [modelName '/Step'], ...
    'Position', [40 80 70 110], ...
    'Time', '0', 'Before', '0', 'After', '1');

add_block('simulink/Discrete/Discrete Transfer Fcn', [modelName '/Gz'], ...
    'Position', [150 70 300 120]);

add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
    'Position', [380 75 430 115]);

% Parameters
set_param([modelName '/Gz'], ...
    'Numerator', '[0.0676 0.0604]', ...
    'Denominator', '[1 -1.5770 0.6724]', ...
    'SampleTime', '0.1');

% Connect lines
add_line(modelName, 'Step/1', 'Gz/1', 'autorouting', 'on');
add_line(modelName, 'Gz/1', 'Scope/1', 'autorouting', 'on');

% Simulation settings
set_param(modelName, 'StopTime', '10');
save_system(modelName);
disp(['Created model: ' modelName]);

% To run:
% sim(modelName);
