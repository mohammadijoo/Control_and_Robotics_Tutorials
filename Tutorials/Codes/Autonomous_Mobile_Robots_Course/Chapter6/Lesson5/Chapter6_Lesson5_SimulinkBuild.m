% Chapter6_Lesson5_SimulinkBuild.m
% Programmatically build a minimal Simulink diagram for the Bayes filter loop.
% Output model: Chapter6_Lesson5_BayesLoop.slx
%
% Note: This is a pedagogical wiring diagram (not a full estimator blockset).
% It shows the loop structure: Prediction -> Likelihood -> Product -> Normalize.
%
% Run in MATLAB with Simulink installed.

function Chapter6_Lesson5_SimulinkBuild()

model = 'Chapter6_Lesson5_BayesLoop';
if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model); open_system(model);

% Layout helpers
x0 = 30; y0 = 50; dx = 160; dy = 80;

% Sources
add_block('simulink/Sources/Constant', [model '/bel_prev'], 'Value', 'bel0', ...
    'Position', [x0 y0 x0+80 y0+30]);

add_block('simulink/Sources/Constant', [model '/kernel'], 'Value', 'k', ...
    'Position', [x0 y0+dy x0+80 y0+dy+30]);

add_block('simulink/Sources/Constant', [model '/likelihood'], 'Value', 'l', ...
    'Position', [x0 y0+2*dy x0+80 y0+2*dy+30]);

% Prediction (placeholder): bel_bar = Conv(bel_prev, kernel)
add_block('simulink/Math Operations/Product', [model '/ConvPlaceholder'], ...
    'Position', [x0+dx y0+dy/2 x0+dx+80 y0+dy/2+40]);
set_param([model '/ConvPlaceholder'], 'Inputs', '**'); % used as placeholder

% Update: multiply bel_bar .* likelihood
add_block('simulink/Math Operations/Product', [model '/Multiply'], ...
    'Position', [x0+2*dx y0+dy x0+2*dx+80 y0+dy+40]);
set_param([model '/Multiply'], 'Inputs', '.*'); % elementwise

% Normalize: bel = bel_unnorm / sum(bel_unnorm)
add_block('simulink/Math Operations/Sum', [model '/SumBel'], ...
    'Position', [x0+3*dx y0+dy-10 x0+3*dx+60 y0+dy+10]);
set_param([model '/SumBel'], 'Inputs', '+');

add_block('simulink/Math Operations/Divide', [model '/Normalize'], ...
    'Position', [x0+3*dx y0+dy+30 x0+3*dx+80 y0+dy+70]);

% Sink
add_block('simulink/Sinks/Out1', [model '/bel'], ...
    'Position', [x0+4*dx y0+dy+40 x0+4*dx+60 y0+dy+60]);

% Connections
add_line(model, 'bel_prev/1', 'ConvPlaceholder/1');
add_line(model, 'kernel/1',   'ConvPlaceholder/2');

add_line(model, 'ConvPlaceholder/1', 'Multiply/1');
add_line(model, 'likelihood/1',      'Multiply/2');

add_line(model, 'Multiply/1', 'SumBel/1');
add_line(model, 'Multiply/1', 'Normalize/1');
add_line(model, 'SumBel/1',   'Normalize/2');

add_line(model, 'Normalize/1', 'bel/1');

% Clean
set_param(model, 'StopTime', '1');
save_system(model);
disp(['Saved model: ' model '.slx']);

end
