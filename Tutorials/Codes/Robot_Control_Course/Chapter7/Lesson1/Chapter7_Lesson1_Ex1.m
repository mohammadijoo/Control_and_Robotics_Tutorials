
% Create a new Simulink model
model = 'joint_uncertain_model';
new_system(model);
open_system(model);

% Add blocks (simplified; positions omitted)
add_block('simulink/Sources/Step', [model '/tau_ref']);
add_block('simulink/Sources/Step', [model '/disturbance']);
add_block('simulink/Continuous/State-Space', [model '/joint_dyn']);

% Set state-space parameters using A,B,E
set_param([model '/joint_dyn'], 'A', mat2str(A), ...
                               'B', mat2str([B E]), ...
                               'C', 'eye(2)', ...
                               'D', 'zeros(2,2)');

% Connect blocks as needed and run:
set_param(model, 'StopTime', '5');
sim(model);
