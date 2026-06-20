% ===== Code block 1 extracted from Chapter5/Lesson3.html =====
% Matrices from Section 5
A = [0 1; -2 -3];
B = [0 0; 4 -1];
C = [1 0; 0 1];
D = [0 0; 0 0.5];

sys = ss(A,B,C,D);

t = linspace(0,10,2001);
u1 = ones(size(t));
u2 = exp(-0.7*t);
U = [u1(:) u2(:)];   % N-by-m

x0 = [0; 0];
[y, tOut, x] = lsim(sys, U, t, x0);

% y is N-by-p, x is N-by-n
disp(size(y));
disp(size(x));

% Verify y2 = x2 + 0.5*u2
y2check = x(:,2) + 0.5*u2(:);
fprintf("max|y2 - (x2+0.5u2)| = %.3e\n", max(abs(y(:,2) - y2check)));
      

% ===== Code block 2 extracted from Chapter5/Lesson3.html =====
modelName = 'mimo_state_space_demo';
new_system(modelName); open_system(modelName);

% Add input sources
add_block('simulink/Sources/Step', [modelName '/u1_step'], 'Position', [50 50 80 80]);
add_block('simulink/Sources/Clock', [modelName '/clock'], 'Position', [50 130 80 160]);
add_block('simulink/Math Operations/Gain', [modelName '/gain_minus0p7'], ...
  'Gain', '-0.7', 'Position', [120 130 170 160]);
add_block('simulink/Math Operations/Math Function', [modelName '/exp'], ...
  'Operator', 'exp', 'Position', [200 130 250 160]);

% Mux inputs to 2-channel vector
add_block('simulink/Signal Routing/Mux', [modelName '/Mux'], 'Inputs', '2', 'Position', [300 70 320 150]);

% Add State-Space block
add_block('simulink/Continuous/State-Space', [modelName '/StateSpace'], 'Position', [380 80 480 140]);

A = [0 1; -2 -3];
B = [0 0; 4 -1];
C = [1 0; 0 1];
D = [0 0; 0 0.5];

set_param([modelName '/StateSpace'], 'A', mat2str(A), 'B', mat2str(B), 'C', mat2str(C), 'D', mat2str(D));

% Add Scope
add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [540 90 570 120]);

% Wiring
add_line(modelName, 'u1_step/1', 'Mux/1');
add_line(modelName, 'clock/1', 'gain_minus0p7/1');
add_line(modelName, 'gain_minus0p7/1', 'exp/1');
add_line(modelName, 'exp/1', 'Mux/2');
add_line(modelName, 'Mux/1', 'StateSpace/1');
add_line(modelName, 'StateSpace/1', 'Scope/1');

save_system(modelName);
      
