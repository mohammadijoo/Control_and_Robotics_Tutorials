% ===== Code block 1 extracted from Chapter3/Lesson1.html =====
% MATLAB: rank and null space
A = [1 2 3 1;
     2 4 6 2;
     1 1 1 0];

r = rank(A);          % numerical rank
N = null(A,'r');      % rational basis if possible (symbolic-style), otherwise uses tolerance
disp(['rank(A) = ' num2str(r)]);
disp(['nullity(A) = ' num2str(size(N,2))]);
disp(['rank + nullity = ' num2str(r + size(N,2)) ' (should be n = ' num2str(size(A,2)) ')']);

disp('Frobenius norm ||A*N||_F = ');
disp(norm(A*N,'fro'));

% Exact row-reduction (RREF) for pedagogy
[R, piv] = rref(A);
disp('RREF(A) = '); disp(R);
disp('Pivot columns = '); disp(piv);

% ===== Code block 2 extracted from Chapter3/Lesson1.html =====
% Build a minimal Simulink model that computes rank(A(t)) inside a MATLAB Function block
model = 'mc_rank_null_demo';
new_system(model); open_system(model);

add_block('simulink/Sources/Clock',[model '/Clock']);
add_block('simulink/User-Defined Functions/MATLAB Function',[model '/RankBlock']);

set_param([model '/RankBlock'], 'Script', sprintf([ ...
'function r = fcn(t)\n' ...
'%% Example time-varying matrix A(t)\n' ...
'A = [1 2 3 1; 2 4 6 2; 1 1 1 0] + 1e-3*sin(t)*[1 0 0 0; 0 0 0 0; 0 0 0 0];\n' ...
'r = rank(A);\n' ...
'end\n'
]));

add_block('simulink/Sinks/Display',[model '/Display']);

add_line(model,'Clock/1','RankBlock/1');
add_line(model,'RankBlock/1','Display/1');

set_param(model,'StopTime','10');
save_system(model);
