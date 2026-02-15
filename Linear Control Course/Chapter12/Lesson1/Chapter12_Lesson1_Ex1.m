% Create new Simulink model
modelName = 'PID_from_time_specs';
new_system(modelName);
open_system(modelName);

% Add blocks: Step, Sum, PID, Transfer Fcn, Scope
add_block('simulink/Sources/Step', [modelName '/Step']);
add_block('simulink/Math Operations/Sum', [modelName '/Sum'], ...
          'Inputs', '+-');
add_block('simulink/Continuous/PID Controller', [modelName '/PID']);
add_block('simulink/Continuous/Transfer Fcn', [modelName '/Plant']);
add_block('simulink/Sinks/Scope', [modelName '/Scope']);

% Set PID parameters
set_param([modelName '/PID'], 'P', num2str(Kp), ...
                             'I', num2str(Ki), ...
                             'D', num2str(Kd));

% Set plant transfer function 1/(s*(s+1)) => num=1, den=[1 1 0]
set_param([modelName '/Plant'], 'Numerator', '[1]', ...
                                 'Denominator', '[1 1 0]');

% Connect blocks (positions omitted for brevity)
add_line(modelName, 'Step/1', 'Sum/1');
add_line(modelName, 'Sum/1',  'PID/1');
add_line(modelName, 'PID/1',  'Plant/1');
add_line(modelName, 'Plant/1','Scope/1');
add_line(modelName, 'Plant/1','Sum/2');

save_system(modelName);
