% ===== Code block 1 extracted from Chapter6/Lesson4.html =====
s = tf('s');

G1 = (s+1)/(s+2);
G2 = (s+2)/(s+3);

G_series = series(G2, G1);    % cascade
G_min    = minreal(G_series); % cancellation (tolerance-based)

disp('G_series ='); disp(G_series);
disp('G_min =');    disp(G_min);

% State-space realizations and orders
S_series = ss(G_series);
S_min    = ss(G_min);

disp(['Order(series realization) = ', num2str(order(S_series))]);
disp(['Order(minreal)            = ', num2str(order(S_min))]);

% Compare step responses (outputs match)
figure; step(G_series, G_min); grid on;
legend('Cascade (with cancellation)', 'Reduced (minreal)');

% Inspect internal states of the cascade realization
% (Note: the internal states depend on the realization; this is a demonstration.)
A = S_series.A; B = S_series.B; C = S_series.C; D = S_series.D;
disp('A matrix (series realization) ='); disp(A);
      

% ===== Code block 2 extracted from Chapter6/Lesson4.html =====
modelName = 'ch6_l4_cascade_cancellation';
new_system(modelName); open_system(modelName);

% Blocks
add_block('simulink/Sources/Step', [modelName, '/Step']);
add_block('simulink/Continuous/Transfer Fcn', [modelName, '/G1']);
add_block('simulink/Continuous/Transfer Fcn', [modelName, '/G2']);
add_block('simulink/Continuous/Transfer Fcn', [modelName, '/Gmin']);
add_block('simulink/Sinks/Scope', [modelName, '/Scope']);

% Parameters: (s+1)/(s+2), (s+2)/(s+3), (s+1)/(s+3)
set_param([modelName, '/G1'], 'Numerator', '[1 1]', 'Denominator', '[1 2]');
set_param([modelName, '/G2'], 'Numerator', '[1 2]', 'Denominator', '[1 3]');
set_param([modelName, '/Gmin'], 'Numerator', '[1 1]', 'Denominator', '[1 3]');

% Wiring: Step -> G1 -> G2 -> Scope (input 1), and Step -> Gmin -> Scope (input 2)
add_line(modelName, 'Step/1', 'G1/1');
add_line(modelName, 'G1/1', 'G2/1');
add_line(modelName, 'G2/1', 'Scope/1');

add_line(modelName, 'Step/1', 'Gmin/1');
add_line(modelName, 'Gmin/1', 'Scope/2');

set_param(modelName, 'StopTime', '10');
save_system(modelName);
      
