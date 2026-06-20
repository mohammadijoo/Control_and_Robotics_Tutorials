% Chapter 12 - Lesson 2 (Simulink companion): Bode plot from a Simulink model
% System Dynamics (Control Engineering)
%
% This script programmatically builds a simple Simulink model of a transfer function
% and opens a Bode plot via linearization (Simulink Control Design recommended).
%
% Requirements:
%  - Simulink
%  - (Recommended) Simulink Control Design for linearize/linio

clear; clc;

model = 'Chapter12_Lesson2_BodeModel';
if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model);
open_system(model);

% Blocks
add_block('simulink/Sources/In1', [model '/u'], 'Position', [30 80 60 100]);
add_block('simulink/Continuous/Transfer Fcn', [model '/G'], 'Position', [120 70 230 110]);
add_block('simulink/Sinks/Out1', [model '/y'], 'Position', [300 80 330 100]);

% Example G(s) = 10*(s+1) / (0.1*s^2 + s)
set_param([model '/G'], 'Numerator', '[10 10]', 'Denominator', '[0.1 1 0]');

% Lines
add_line(model, 'u/1', 'G/1');
add_line(model, 'G/1', 'y/1');

set_param(model, 'StopTime', '10');

% Define linearization I/O points (if available)
try
    io(1) = linio([model '/u'], 1, 'input');
    io(2) = linio([model '/y'], 1, 'output');
    sys = linearize(model, io);
    figure; bode(sys); grid on;
    title('Bode from linearized Simulink model');
catch ME
    warning('Linearization could not be performed. Details:\n%s', ME.message);
    disp('Alternative: Use Simulink "Linear Analysis" tool to define I/O and plot Bode.');
end
