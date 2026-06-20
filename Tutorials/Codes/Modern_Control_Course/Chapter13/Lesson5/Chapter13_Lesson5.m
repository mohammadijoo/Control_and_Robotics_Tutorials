% Chapter13_Lesson5.m
% Examples of observable and unobservable systems.
% Requires base MATLAB. The Control System Toolbox function obsv is optional.

clear; clc; close all;

examples = {};

examples{end+1}.name = 'Example 1: observable second-order system';
examples{end}.A = [0 1; -2 -3];
examples{end}.C = [1 0];

examples{end+1}.name = 'Example 2: unobservable and not detectable';
examples{end}.A = [-1 0; 0 2];
examples{end}.C = [1 0];

examples{end+1}.name = 'Example 3: unobservable but detectable';
examples{end}.A = [-1 0; 0 -4];
examples{end}.C = [1 0];

examples{end+1}.name = 'Example 4a: velocity sensor';
examples{end}.A = [0 1; -6 -5];
examples{end}.C = [0 1];

examples{end+1}.name = 'Example 4b: position sensor';
examples{end}.A = [0 1; -6 -5];
examples{end}.C = [1 0];

for k = 1:numel(examples)
    A = examples{k}.A;
    C = examples{k}.C;
    O = local_obsv(A, C);
    r = rank(O);
    n = size(A, 1);

    fprintf('\n===============================================================\n');
    fprintf('%s\n', examples{k}.name);
    disp('A ='); disp(A);
    disp('C ='); disp(C);
    disp('O ='); disp(O);
    fprintf('rank(O) = %d out of n = %d\n', r, n);
    disp('eig(A) ='); disp(eig(A).');

    if r == n
        fprintf('Conclusion: observable.\n');
    else
        fprintf('Conclusion: unobservable. Inspect hidden modes for detectability.\n');
    end
end

% Demonstrate indistinguishable outputs for Example 2.
A = [-1 0; 0 2];
C = [1 0];
t = linspace(0, 3, 250);
x0a = [1; 0];
x0b = [1; 5];
ya = zeros(size(t));
yb = zeros(size(t));

for i = 1:numel(t)
    ya(i) = C * expm(A * t(i)) * x0a;
    yb(i) = C * expm(A * t(i)) * x0b;
end

figure;
plot(t, ya, 'LineWidth', 1.5); hold on;
plot(t, yb, '--', 'LineWidth', 1.5);
grid on;
xlabel('time');
ylabel('output y(t)');
title('Unobservable Example: Different Initial States, Same Output');
legend('x0 = [1;0]', 'x0 = [1;5]', 'Location', 'best');

function O = local_obsv(A, C)
    n = size(A, 1);
    O = [];
    Ak = eye(n);
    for k = 1:n
        O = [O; C * Ak];
        Ak = Ak * A;
    end
end
