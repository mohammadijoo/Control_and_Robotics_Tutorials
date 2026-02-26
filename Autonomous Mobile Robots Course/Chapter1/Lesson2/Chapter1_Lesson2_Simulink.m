% Chapter1_Lesson2_Simulink.m
% Autonomous Mobile Robots — Chapter 1, Lesson 2
% Create a Simulink model that integrates planar pose
%   xdot = v*cos(theta), ydot = v*sin(theta), thetadot = omega

clear; clc;

model = 'Chapter1_Lesson2_Simulink';
if bdIsLoaded(model)
    close_system(model, 0);
end
if exist([model '.slx'], 'file')
    delete([model '.slx']);
end

new_system(model);
open_system(model);

set_param(model, 'StopTime', '20');

add_block('simulink/Sources/Constant', [model '/v'], 'Value', '0.8', 'Position', [40 50 90 80]);
add_block('simulink/Sources/Constant', [model '/omega'], 'Value', '0.35', 'Position', [40 150 90 180]);

add_block('simulink/Continuous/Integrator', [model '/Int_x'], 'InitialCondition', '0', 'Position', [520 40 550 70]);
add_block('simulink/Continuous/Integrator', [model '/Int_y'], 'InitialCondition', '0', 'Position', [520 100 550 130]);
add_block('simulink/Continuous/Integrator', [model '/Int_theta'], 'InitialCondition', '0', 'Position', [240 150 270 180]);

add_block('simulink/Math Operations/Trigonometric Function', [model '/cos'], 'Operator', 'cos', 'Position', [300 40 360 70]);
add_block('simulink/Math Operations/Trigonometric Function', [model '/sin'], 'Operator', 'sin', 'Position', [300 100 360 130]);

add_block('simulink/Math Operations/Product', [model '/v_cos'], 'Position', [400 40 460 70]);
add_block('simulink/Math Operations/Product', [model '/v_sin'], 'Position', [400 100 460 130]);

add_block('simulink/Sinks/To Workspace', [model '/x_out'], 'VariableName', 'x_out', 'SaveFormat', 'StructureWithTime', 'Position', [600 40 680 70]);
add_block('simulink/Sinks/To Workspace', [model '/y_out'], 'VariableName', 'y_out', 'SaveFormat', 'StructureWithTime', 'Position', [600 100 680 130]);
add_block('simulink/Sinks/To Workspace', [model '/theta_out'], 'VariableName', 'theta_out', 'SaveFormat', 'StructureWithTime', 'Position', [360 150 460 180]);

add_line(model, 'omega/1', 'Int_theta/1');
add_line(model, 'Int_theta/1', 'cos/1');
add_line(model, 'Int_theta/1', 'sin/1');

add_line(model, 'v/1', 'v_cos/1');
add_line(model, 'cos/1', 'v_cos/2');

add_line(model, 'v/1', 'v_sin/1');
add_line(model, 'sin/1', 'v_sin/2');

add_line(model, 'v_cos/1', 'Int_x/1');
add_line(model, 'v_sin/1', 'Int_y/1');

add_line(model, 'Int_x/1', 'x_out/1');
add_line(model, 'Int_y/1', 'y_out/1');
add_line(model, 'Int_theta/1', 'theta_out/1');

save_system(model);

disp(['Created and saved Simulink model: ', model, '.slx']);
disp('Run: sim(model) then inspect x_out, y_out, theta_out in workspace.');
