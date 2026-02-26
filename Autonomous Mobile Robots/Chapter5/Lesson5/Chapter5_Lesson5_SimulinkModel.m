% Chapter5_Lesson5_SimulinkModel.m
%
% Creates a basic Simulink model to integrate differential-drive odometry from
% wheel encoder increments.
%
% The model:
% - Inputs: dphiL, dphiR, theta (previous), and parameters rL, rR, b
% - Computes ds, dtheta and integrates to x,y,theta using midpoint integration
%
% Notes:
% - This is a model builder script. It programmatically creates blocks in a new model.
% - Requires Simulink.
%
% Usage:
%   Chapter5_Lesson5_SimulinkModel
%
function Chapter5_Lesson5_SimulinkModel

model = 'Chapter5_Lesson5_OdometryModel';
new_system(model);
open_system(model);

% Add Inports
add_block('simulink/Sources/In1', [model '/dphiL'], 'Position', [30 50 60 70]);
add_block('simulink/Sources/In1', [model '/dphiR'], 'Position', [30 100 60 120]);
add_block('simulink/Sources/In1', [model '/theta_prev'], 'Position', [30 150 60 170]);

% Parameters as Constant blocks (edit values as needed)
add_block('simulink/Sources/Constant', [model '/rL'], 'Value', '0.05', 'Position', [30 220 80 240]);
add_block('simulink/Sources/Constant', [model '/rR'], 'Value', '0.05', 'Position', [30 260 80 280]);
add_block('simulink/Sources/Constant', [model '/b'],  'Value', '0.30', 'Position', [30 300 80 320]);

% Compute sL = rL*dphiL, sR = rR*dphiR
add_block('simulink/Math Operations/Product', [model '/sL'], 'Position', [140 50 170 70]);
add_block('simulink/Math Operations/Product', [model '/sR'], 'Position', [140 100 170 120]);

add_line(model, 'dphiL/1', 'sL/1');
add_line(model, 'rL/1', 'sL/2');
add_line(model, 'dphiR/1', 'sR/1');
add_line(model, 'rR/1', 'sR/2');

% ds = 0.5*(sR + sL)
add_block('simulink/Math Operations/Add', [model '/sR_plus_sL'], 'Inputs', '++', 'Position', [220 75 250 105]);
add_block('simulink/Math Operations/Gain', [model '/half'], 'Gain', '0.5', 'Position', [290 82 340 98]);

add_line(model, 'sL/1', 'sR_plus_sL/1');
add_line(model, 'sR/1', 'sR_plus_sL/2');
add_line(model, 'sR_plus_sL/1', 'half/1');

% dtheta = (sR - sL)/b
add_block('simulink/Math Operations/Add', [model '/sR_minus_sL'], 'Inputs', '+-', 'Position', [220 140 250 170]);
add_block('simulink/Math Operations/Divide', [model '/dtheta'], 'Position', [300 145 340 165]);

add_line(model, 'sR/1', 'sR_minus_sL/1');
add_line(model, 'sL/1', 'sR_minus_sL/2');
add_line(model, 'sR_minus_sL/1', 'dtheta/1');
add_line(model, 'b/1', 'dtheta/2');

% theta_mid = theta_prev + 0.5*dtheta
add_block('simulink/Math Operations/Gain', [model '/half_dtheta'], 'Gain', '0.5', 'Position', [380 145 430 165]);
add_block('simulink/Math Operations/Add', [model '/theta_mid'], 'Inputs', '++', 'Position', [470 145 500 165]);

add_line(model, 'dtheta/1', 'half_dtheta/1');
add_line(model, 'theta_prev/1', 'theta_mid/1');
add_line(model, 'half_dtheta/1', 'theta_mid/2');

% dx = ds*cos(theta_mid), dy = ds*sin(theta_mid)
add_block('simulink/Math Operations/Trigonometric Function', [model '/cos'], 'Operator', 'cos', 'Position', [540 135 580 175]);
add_block('simulink/Math Operations/Trigonometric Function', [model '/sin'], 'Operator', 'sin', 'Position', [540 185 580 225]);

add_block('simulink/Math Operations/Product', [model '/dx'], 'Position', [620 145 650 165]);
add_block('simulink/Math Operations/Product', [model '/dy'], 'Position', [620 195 650 215]);

add_line(model, 'theta_mid/1', 'cos/1');
add_line(model, 'theta_mid/1', 'sin/1');
add_line(model, 'half/1', 'dx/1');   % ds
add_line(model, 'cos/1', 'dx/2');
add_line(model, 'half/1', 'dy/1');   % ds
add_line(model, 'sin/1', 'dy/2');

% Integrators for x,y,theta
add_block('simulink/Continuous/Integrator', [model '/Int_x'], 'Position', [720 140 750 170]);
add_block('simulink/Continuous/Integrator', [model '/Int_y'], 'Position', [720 190 750 220]);
add_block('simulink/Continuous/Integrator', [model '/Int_theta'], 'Position', [720 240 750 270]);

add_line(model, 'dx/1', 'Int_x/1');
add_line(model, 'dy/1', 'Int_y/1');
add_line(model, 'dtheta/1', 'Int_theta/1');

% Outputs
add_block('simulink/Sinks/Out1', [model '/x'], 'Position', [800 150 830 170]);
add_block('simulink/Sinks/Out1', [model '/y'], 'Position', [800 200 830 220]);
add_block('simulink/Sinks/Out1', [model '/theta'], 'Position', [800 250 830 270]);

add_line(model, 'Int_x/1', 'x/1');
add_line(model, 'Int_y/1', 'y/1');
add_line(model, 'Int_theta/1', 'theta/1');

save_system(model);
disp(['Created Simulink model: ' model]);

end
