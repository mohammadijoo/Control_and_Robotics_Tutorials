% Chapter16_Lesson4.m
% Example systems in controllable canonical form (CCF).
%
% Requires only base MATLAB for the from-scratch computations.
% The optional ss/tf/ctrb section requires Control System Toolbox.

clear; clc;

examples = {
    'Example 1: G1(s) = 2/(s^2 + 3s + 2)', [1 3 2], [2];
    'Example 2: G2(s) = (s + 2)/(s^3 + 6s^2 + 11s + 6)', [1 6 11 6], [1 2];
    'Example 3: G3(s) = (0.5s^2 + 1.5s + 1)/(s^3 + 4s^2 + 5s + 2)', [1 4 5 2], [0.5 1.5 1.0]
};

for k = 1:size(examples, 1)
    name = examples{k, 1};
    den = examples{k, 2};
    num = examples{k, 3};

    [A, B, C, D] = ccf_from_tf(den, num);
    Wc = controllability_matrix(A, B);

    fprintf('\n%s\n', repmat('=', 1, 72));
    disp(name);
    disp('A ='); disp(A);
    disp('B ='); disp(B);
    disp('C ='); disp(C);
    disp('D ='); disp(D);
    disp('Wc ='); disp(Wc);
    fprintf('rank(Wc) = %d\n', rank(Wc));
    fprintf('det(Wc)  = %.6g\n', det(Wc));

    if exist('ss', 'file') == 2 && exist('tf', 'file') == 2
        sys = ss(A, B, C, D);
        fprintf('Transfer function from ss(A,B,C,D):\n');
        tf(sys)
    end
end

% Optional Simulink generation: creates a simple continuous-time state-space
% block model for Example 1 if Simulink is available.
if exist('new_system', 'file') == 2
    [A1, B1, C1, D1] = ccf_from_tf([1 3 2], [2]);
    modelName = 'Chapter16_Lesson4_Simulink_CCF';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);
    add_block('simulink/Sources/Step', [modelName '/Step']);
    add_block('simulink/Continuous/State-Space', [modelName '/CCF_StateSpace']);
    add_block('simulink/Sinks/Scope', [modelName '/Scope']);
    set_param([modelName '/CCF_StateSpace'], ...
        'A', mat2str(A1), 'B', mat2str(B1), 'C', mat2str(C1), 'D', mat2str(D1));
    add_line(modelName, 'Step/1', 'CCF_StateSpace/1');
    add_line(modelName, 'CCF_StateSpace/1', 'Scope/1');
    set_param(modelName, 'StopTime', '10');
    save_system(modelName);
    fprintf('\nSimulink model saved as %s.slx\n', modelName);
end

function [A, B, C, D] = ccf_from_tf(den_desc, num_desc)
    if abs(den_desc(1)) < 1e-14
        error('Leading denominator coefficient must be nonzero.');
    end

    lead = den_desc(1);
    den_desc = den_desc / lead;
    num_desc = num_desc / lead;
    n = length(den_desc) - 1;

    if length(num_desc) > n
        error('Only strictly proper systems are handled.');
    end

    A = zeros(n, n);
    if n > 1
        A(1:n-1, 2:n) = eye(n - 1);
    end
    A(n, :) = -fliplr(den_desc(2:end));

    B = zeros(n, 1);
    B(n) = 1;

    padded = [zeros(1, n - length(num_desc)), num_desc];
    C = fliplr(padded);
    D = 0;
end

function Wc = controllability_matrix(A, B)
    n = size(A, 1);
    Wc = zeros(n, n);
    Ak = eye(n);
    for k = 1:n
        Wc(:, k) = Ak * B;
        Ak = A * Ak;
    end
end
