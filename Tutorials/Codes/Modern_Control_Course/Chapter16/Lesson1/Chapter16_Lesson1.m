% Chapter16_Lesson1.m
% Companion Matrix and Controllable Canonical Form (CCF)
% Convention:
%   p(s)=s^n+a_{n-1}s^{n-1}+...+a_1 s+a_0
%   a=[a0 a1 ... a_{n-1}]
%   q(s)=b0+b1 s+...+b_{n-1}s^{n-1}
%   b=[b0 b1 ... b_{n-1}]

clear; clc;

a = [2 3 4];   % a0, a1, a2
b = [5 6 7];   % b0, b1, b2
n = length(a);

A = zeros(n);
A(1:n-1, 2:n) = eye(n-1);
A(n, :) = -a;

B = zeros(n, 1);
B(n) = 1;

C = b;
D = 0;

Qc = ctrb(A, B);

disp('A ='); disp(A);
disp('B ='); disp(B);
disp('C ='); disp(C);
disp('D ='); disp(D);
disp('Q_c ='); disp(Qc);
fprintf('rank(Q_c) = %d\n', rank(Qc));
fprintf('det(Q_c)  = %.6g\n', det(Qc));

% Control System Toolbox verification
sys = ss(A, B, C, D);
G = tf(sys);
disp('Transfer function from CCF realization:');
G

% The expected transfer function is:
% (5 + 6s + 7s^2)/(s^3 + 4s^2 + 3s + 2)
expected = tf(fliplr(b), [1 fliplr(a)]);
disp('Expected transfer function:');
expected

% Optional Simulink model construction, if Simulink is available.
if exist('new_system', 'file') == 2
    model = 'Chapter16_Lesson1_Simulink_CCF';
    if bdIsLoaded(model)
        close_system(model, 0);
    end

    new_system(model);
    open_system(model);

    add_block('simulink/Sources/Step', [model '/Step']);
    add_block('simulink/Continuous/State-Space', [model '/CCF State Space']);
    add_block('simulink/Sinks/Scope', [model '/Scope']);

    set_param([model '/CCF State Space'], ...
        'A', 'A', 'B', 'B', 'C', 'C', 'D', 'D');

    add_line(model, 'Step/1', 'CCF State Space/1');
    add_line(model, 'CCF State Space/1', 'Scope/1');

    save_system(model);
    disp(['Created Simulink model: ' model '.slx']);
end
