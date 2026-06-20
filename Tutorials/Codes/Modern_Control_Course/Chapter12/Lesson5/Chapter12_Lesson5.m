% Chapter12_Lesson5.m
% Balanced-realization preview for a continuous-time stable LTI system.
% Requires Control System Toolbox for lyap, ss, and optional Simulink model setup.

clear; clc; close all;

A = [-1.0  0.3;
      0.0 -2.0];
B = [1.0; 0.5];
C = [1.0 -0.2];
D = 0;

% Controllability Gramian:
% A*Wc + Wc*A' + B*B' = 0
Wc = lyap(A, B*B');

% Output-energy / observability Gramian preview:
% A'*Wo + Wo*A + C'*C = 0
Wo = lyap(A', C'*C);

Wc = 0.5*(Wc + Wc');
Wo = 0.5*(Wo + Wo');

% Stable computation of Hankel singular values and balancing transform.
R = chol(Wc, 'lower');
[V, Lambda] = eig(R' * Wo * R);
lambda = diag(Lambda);
[lambda, idx] = sort(lambda, 'descend');
V = V(:, idx);
sigma = sqrt(max(lambda, 0));
Sigma = diag(sigma);

% Balancing transformation for x = T*z.
T = R * V * diag(1 ./ sqrt(sigma));
Tinv = inv(T);

Ab = Tinv * A * T;
Bb = Tinv * B;
Cb = C * T;

Wc_bal = Tinv * Wc * Tinv';
Wo_bal = T' * Wo * T;

disp('Controllability Gramian Wc ='); disp(Wc);
disp('Output-energy Gramian Wo ='); disp(Wo);
disp('Hankel singular values sigma ='); disp(sigma);
disp('Balanced-coordinate Wc ='); disp(Wc_bal);
disp('Balanced-coordinate Wo ='); disp(Wo_bal);
disp('Target Sigma ='); disp(Sigma);

% State-space objects before and after coordinate transformation.
sys_original = ss(A, B, C, D);
sys_balanced = ss(Ab, Bb, Cb, D);

% Simulink setup preview:
% This creates a simple model with one State-Space block using balanced matrices.
make_simulink_model = false;
if make_simulink_model
    mdl = 'Chapter12_Lesson5_Simulink';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    add_block('simulink/Sources/Step', [mdl '/Step Input']);
    add_block('simulink/Continuous/State-Space', [mdl '/Balanced State Space']);
    add_block('simulink/Sinks/Scope', [mdl '/Scope']);

    set_param([mdl '/Balanced State Space'], ...
        'A', mat2str(Ab), ...
        'B', mat2str(Bb), ...
        'C', mat2str(Cb), ...
        'D', mat2str(D));

    add_line(mdl, 'Step Input/1', 'Balanced State Space/1');
    add_line(mdl, 'Balanced State Space/1', 'Scope/1');
    save_system(mdl);
    open_system(mdl);
end
