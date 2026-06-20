% Chapter16_Lesson5.m
% Advantages and drawbacks of controllable canonical form (CCF).
%
% MATLAB libraries/toolboxes related to this lesson:
%   Control System Toolbox: ss, tf, tf2ss, ctrb, place, canon
%   Simulink: State-Space block for model-level simulation

clear; clc;

% Denominator: s^3 + 6s^2 + 11s + 6
% Numerator:   2s^2 + 5s + 3
a = [6 11 6];          % [a0 a1 a2]
beta = [2 5 3];        % [beta0 beta1 beta2], descending numerator powers
n = length(a);

A = zeros(n);
A(1:n-1,2:n) = eye(n-1);
A(n,:) = -a;
B = zeros(n,1); B(n) = 1;
C = fliplr(beta);
D = 0;

fprintf('A_c =\n'); disp(A);
fprintf('B_c =\n'); disp(B);
fprintf('C_c =\n'); disp(C);

Qc = ctrb(A,B);
fprintf('rank(Qc) = %d\n', rank(Qc));
fprintf('cond(Qc) = %.4e\n', cond(Qc));

sys = ss(A,B,C,D);
fprintf('Transfer function reconstructed from CCF:\n');
disp(tf(sys));

% Direct CCF pole placement.
desiredPoles = [-2 -3 -4];
desiredPoly = poly(desiredPoles);       % [1 alpha2 alpha1 alpha0]
alphaAscending = fliplr(desiredPoly(2:end));
K = alphaAscending - a;
Acl = A - B*K;
fprintf('K =\n'); disp(K);
fprintf('eig(A-BK) =\n'); disp(eig(Acl));

% Comparison with MATLAB place.
K_place = place(A,B,desiredPoles);
fprintf('K from place(A,B,p) =\n'); disp(K_place);

% Simulink construction: creates a State-Space block using the CCF matrices.
if exist('simulink', 'file') == 4 || exist('new_system', 'file') == 2
    model = 'Chapter16_Lesson5_Simulink';
    if bdIsLoaded(model), close_system(model,0); end
    new_system(model);
    add_block('simulink/Sources/Step', [model '/Reference Step'], ...
        'Position', [40 80 90 110]);
    add_block('simulink/Continuous/State-Space', [model '/CCF State-Space'], ...
        'A', mat2str(Acl), 'B', mat2str(B), 'C', mat2str(C), 'D', mat2str(D), ...
        'Position', [150 65 310 125]);
    add_block('simulink/Sinks/Scope', [model '/Output Scope'], ...
        'Position', [380 75 430 115]);
    add_line(model, 'Reference Step/1', 'CCF State-Space/1');
    add_line(model, 'CCF State-Space/1', 'Output Scope/1');
    save_system(model);
    fprintf('Created Simulink model: %s.slx\n', model);
else
    fprintf('Simulink not available; skipped model construction.\n');
end
