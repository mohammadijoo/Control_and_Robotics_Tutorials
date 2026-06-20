% ===== Code block 1 extracted from Chapter1/Lesson5.html =====
A = [0 1; -2 -3];
B = [0 1; 1 0];
C = eye(2);
D = zeros(2,2);

sys = ss(A,B,C,D);
lam = eig(A)

% Inspect dimensions (n,m,p)
size(A)  % n x n
size(B)  % n x m
size(C)  % p x n
size(D)  % p x m
      

% ===== Code block 2 extracted from Chapter1/Lesson5.html =====
modelName = 'mc_lesson5_statespace_demo';
new_system(modelName);
open_system(modelName);

A = [0 1; -2 -3];
B = [0 1; 1 0];
C = eye(2);
D = zeros(2,2);

add_block('simulink/Sources/Step', [modelName '/Step1'], 'Position', [30 50 60 80]);
add_block('simulink/Sources/Step', [modelName '/Step2'], 'Position', [30 120 60 150]);
add_block('simulink/Signal Routing/Mux', [modelName '/Mux'], 'Inputs', '2', 'Position', [100 65 120 135]);

add_block('simulink/Continuous/State-Space', [modelName '/StateSpace'], ...
          'A', mat2str(A), 'B', mat2str(B), 'C', mat2str(C), 'D', mat2str(D), ...
          'Position', [170 70 260 130]);

add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [310 75 340 125]);

add_line(modelName, 'Step1/1', 'Mux/1');
add_line(modelName, 'Step2/1', 'Mux/2');
add_line(modelName, 'Mux/1', 'StateSpace/1');
add_line(modelName, 'StateSpace/1', 'Scope/1');

set_param(modelName, 'StopTime', '10');
save_system(modelName);
      
