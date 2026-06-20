% ===== Code block 1 extracted from Chapter2/Lesson1.html =====
% Candidate vectors as columns
V = [1 2 0 1;
     0 1 1 1;
     1 3 1 2;
     0 0 1 1];

[R, piv] = rref(V);   % piv: pivot column indices
dimSpan = numel(piv);
B = V(:, piv);

disp('Pivot columns:'); disp(piv);
disp('Dimension of span:'); disp(dimSpan);
disp('Basis columns:'); disp(B);
disp('RREF:'); disp(R);

% Coordinate representation (solve B*c = v if v is in span(B))
v = [3;2;5;2];
c = B \ v;            % exact if B is square/invertible; otherwise use least squares below
c_ls = lsqr(B, v);    % least squares coordinates
disp('Least-squares coordinates:'); disp(c_ls);
disp('Residual norm:'); disp(norm(B*c_ls - v));
      

% ===== Code block 2 extracted from Chapter2/Lesson1.html =====
% Programmatically create a minimal Simulink model that forms a 2-vector and scales it
model = 'vector_space_demo';
new_system(model); open_system(model);

add_block('simulink/Sources/Sine Wave', [model '/s1']);
add_block('simulink/Sources/Sine Wave', [model '/s2']);
add_block('simulink/Signal Routing/Mux', [model '/Mux'], 'Inputs', '2');
add_block('simulink/Math Operations/Gain', [model '/MatrixGain'], 'Gain', '[2 0; 0 0.5]');
add_block('simulink/Sinks/Scope', [model '/Scope']);

set_param([model '/s1'], 'Position', [30 40 60 60]);
set_param([model '/s2'], 'Position', [30 100 60 120]);
set_param([model '/Mux'], 'Position', [120 55 150 115]);
set_param([model '/MatrixGain'], 'Position', [210 70 260 100]);
set_param([model '/Scope'], 'Position', [320 70 350 100]);

add_line(model, 's1/1', 'Mux/1');
add_line(model, 's2/1', 'Mux/2');
add_line(model, 'Mux/1', 'MatrixGain/1');
add_line(model, 'MatrixGain/1', 'Scope/1');

save_system(model);
      
