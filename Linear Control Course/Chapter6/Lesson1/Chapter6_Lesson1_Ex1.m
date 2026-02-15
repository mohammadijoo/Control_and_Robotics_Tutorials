model = 'second_order_model';
new_system(model);
open_system(model);

add_block('simulink/Sources/Step',        [model '/Step']);
add_block('simulink/Continuous/Transfer Fcn', [model '/G']);
add_block('simulink/Sinks/Scope',         [model '/Scope']);

set_param([model '/G'], 'Numerator',   'K*wn^2', ...
                         'Denominator', '[1 2*zeta*wn wn^2]');

add_line(model, 'Step/1', 'G/1');
add_line(model, 'G/1',    'Scope/1');

set_param(model, 'StopTime', '5');
sim(model);
