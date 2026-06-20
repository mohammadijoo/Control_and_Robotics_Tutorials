% ===== Code block 1 extracted from Chapter4/Lesson3.html =====
% Example matrices
A = [0 1 0;
     0 0 0;
     0 0 -1];
B = [0; 1; 0];
C = [1 0 0];
D = 0;

n = size(A,1);

% Build O = [C; C A; ...; C A^(n-1)]
O = [];
Ak = eye(n);
for k = 1:n
    O = [O; C*Ak];
    Ak = Ak*A;
end

rankO = rank(O);
Ny = null(O,'r');   % rational basis if possible; otherwise numeric

disp('O ='); disp(O);
disp(['rank(O) = ', num2str(rankO)]);
disp('Basis for Ny = ker(O) (columns):'); disp(Ny);

% Create state-space model (requires Control System Toolbox)
sys = ss(A,B,C,D);

% Minimal realization (will remove redundant states if detectable numerically)
sys_min = minreal(sys);

disp('Original order:'); disp(order(sys));
disp('Minimal order (minreal):'); disp(order(sys_min));
      

% ===== Code block 2 extracted from Chapter4/Lesson3.html =====
% Programmatically build a small Simulink model
modelName = 'lesson4_ch4_l3_minimal_demo';
new_system(modelName); open_system(modelName);

add_block('simulink/Continuous/State-Space', [modelName '/FullModel']);
set_param([modelName '/FullModel'], 'A', mat2str(A), 'B', mat2str(B), 'C', mat2str(C), 'D', mat2str(D));

% Reduced model from minreal
[Ar,Br,Cr,Dr] = ssdata(sys_min);
add_block('simulink/Continuous/State-Space', [modelName '/ReducedModel']);
set_param([modelName '/ReducedModel'], 'A', mat2str(Ar), 'B', mat2str(Br), 'C', mat2str(Cr), 'D', mat2str(Dr));

% Add sources/sinks (Step input and Scope)
add_block('simulink/Sources/Step', [modelName '/Step']);
add_block('simulink/Sinks/Scope', [modelName '/ScopeFull']);
add_block('simulink/Sinks/Scope', [modelName '/ScopeReduced']);

% Wire: Step -> both models -> scopes
add_line(modelName, 'Step/1', 'FullModel/1');
add_line(modelName, 'Step/1', 'ReducedModel/1');
add_line(modelName, 'FullModel/1', 'ScopeFull/1');
add_line(modelName, 'ReducedModel/1', 'ScopeReduced/1');

set_param(modelName, 'StopTime', '10');
save_system(modelName);
      
