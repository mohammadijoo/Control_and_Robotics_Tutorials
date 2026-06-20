% Chapter20_Lesson3.m
% Internal vs external equivalence of continuous-time LTI systems.
% Uses only base MATLAB for matrices. The optional Simulink section creates
% a State-Space block if Simulink is installed.

clear; clc;

A1 = [-1 0; 0 -2];
B1 = [1; 1];
C1 = [1 1];
D1 = 0;

% Internal equivalence: x2 = T x1.
T  = [1 2; 0.5 1.5];
A2 = T*A1/T;
B2 = T*B1;
C2 = C1/T;
D2 = D1;

% External equivalence only: add an unreachable and unobservable hidden mode.
A3 = [-1 0 0; 0 -2 0; 0 0 5];
B3 = [1; 1; 0];
C3 = [1 1 0];
D3 = 0;

reportSystem('Sigma_1 minimal', A1, B1, C1, D1);
reportSystem('Sigma_2 internally equivalent to Sigma_1', A2, B2, C2, D2);
reportSystem('Sigma_3 externally equivalent but nonminimal', A3, B3, C3, D3);

fprintf('\nInternal-equivalence residuals for Sigma_1 and Sigma_2:\n');
fprintf('A residual = %.3e\n', norm(A2 - T*A1/T));
fprintf('B residual = %.3e\n', norm(B2 - T*B1));
fprintf('C residual = %.3e\n', norm(C2 - C1/T));
fprintf('D residual = %.3e\n', norm(D2 - D1));

fprintf('\nTransfer-function samples G_i(s):\n');
for s = [0.1 1.0 3.0 10.0]
    g1 = transferValue(A1, B1, C1, D1, s);
    g2 = transferValue(A2, B2, C2, D2, s);
    g3 = transferValue(A3, B3, C3, D3, s);
    fprintf('s=%4.1f  G1=% .8f  G2=% .8f  G3=% .8f\n', s, g1, g2, g3);
end

fprintf('\nFirst Markov parameters D, C B, C A B, ... for Sigma_1 and Sigma_3:\n');
M1 = markovParameters(A1, B1, C1, D1, 6);
M3 = markovParameters(A3, B3, C3, D3, 6);
for k = 1:numel(M1)
    fprintf('k=%d  Sigma_1=% .8f  Sigma_3=% .8f\n', k-1, M1{k}, M3{k});
end

% Optional Simulink generation: create a State-Space block for Sigma_1.
% This demonstrates how the same state-space matrices can be used in Simulink.
try
    modelName = 'Chapter20_Lesson3_Simulink';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);
    assignin('base', 'A1', A1);
    assignin('base', 'B1', B1);
    assignin('base', 'C1', C1);
    assignin('base', 'D1', D1);
    add_block('simulink/Sources/Step', [modelName '/StepInput'], 'Position', [40 80 80 110]);
    add_block('simulink/Continuous/State-Space', [modelName '/MinimalRealization'], 'Position', [140 70 280 120]);
    set_param([modelName '/MinimalRealization'], 'A', 'A1', 'B', 'B1', 'C', 'C1', 'D', 'D1');
    add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [340 75 380 115]);
    add_line(modelName, 'StepInput/1', 'MinimalRealization/1');
    add_line(modelName, 'MinimalRealization/1', 'Scope/1');
    save_system(modelName);
    fprintf('\nSimulink model Chapter20_Lesson3_Simulink.slx was created.\n');
catch ME
    fprintf('\nSimulink section skipped: %s\n', ME.message);
end

function R = ctrbLocal(A, B)
    n = size(A, 1);
    R = [];
    Ak = eye(n);
    for k = 1:n
        R = [R Ak*B]; %#ok<AGROW>
        Ak = A*Ak;
    end
end

function O = obsvLocal(A, C)
    n = size(A, 1);
    O = [];
    Ak = eye(n);
    for k = 1:n
        O = [O; C*Ak]; %#ok<AGROW>
        Ak = Ak*A;
    end
end

function g = transferValue(A, B, C, D, s)
    n = size(A, 1);
    g = C*((s*eye(n) - A)\B) + D;
end

function params = markovParameters(A, B, C, D, count)
    n = size(A, 1);
    params = cell(count + 1, 1);
    params{1} = D;
    Ak = eye(n);
    for k = 1:count
        params{k + 1} = C*Ak*B;
        Ak = A*Ak;
    end
end

function reportSystem(name, A, B, C, D)
    n = size(A, 1);
    rc = rank(ctrbLocal(A, B));
    ro = rank(obsvLocal(A, C));
    fprintf('\n%s\n', name);
    disp('A ='); disp(A);
    disp('B ='); disp(B);
    disp('C ='); disp(C);
    disp('D ='); disp(D);
    fprintf('rank controllability = %d of %d\n', rc, n);
    fprintf('rank observability   = %d of %d\n', ro, n);
    fprintf('minimal? %d\n', rc == n && ro == n);
    fprintf('eigenvalues(A) = '); disp(eig(A).');
end
