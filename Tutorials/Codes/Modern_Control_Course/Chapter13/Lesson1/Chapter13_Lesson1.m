% Chapter13_Lesson1.m
% Intuitive observability from output measurements for continuous-time LTI systems.
% MATLAB Control System Toolbox functions: ss, initial, obsv.
% The first part also works with basic MATLAB except initial/ss/obsv.

clear; clc; close all;

A1 = [0 1; -2 -3];
C1 = [1 0];
x01 = [1; 2];

A2 = [-1 0; 0 -2];
C2 = [1 0];
x02 = [3; 4];

fprintf('\nExample 1: position sensor, observable\n');
O1 = local_observability_matrix(A1, C1);
disp(O1);
fprintf('rank(O1) = %d out of n = %d\n', rank(O1), size(A1,1));

fprintf('\nExample 2: second state invisible, unobservable\n');
O2 = local_observability_matrix(A2, C2);
disp(O2);
fprintf('rank(O2) = %d out of n = %d\n', rank(O2), size(A2,1));

% Derivative-stack reconstruction: z = O_n x0.
z1 = O1*x01;
x01_hat = pinv(O1)*z1;
fprintf('\nTrue x01 = [%g %g]^T\n', x01(1), x01(2));
fprintf('Recovered x01 from y(0), y_dot(0) = [%g %g]^T\n', x01_hat(1), x01_hat(2));

% Compare output traces for two different initial conditions.
t = linspace(0, 5, 300);
Y1 = zeros(size(t));
Y2 = zeros(size(t));
x0a = [1; 2];
x0b = [1; -1];
for k = 1:numel(t)
    Y1(k) = C1*expm(A1*t(k))*x0a;
    Y2(k) = C1*expm(A1*t(k))*x0b;
end

figure;
plot(t, Y1, 'LineWidth', 1.5); hold on;
plot(t, Y2, '--', 'LineWidth', 1.5);
grid on;
xlabel('time');
ylabel('zero-input output y(t)');
legend('x0 = [1, 2]^T', 'x0 = [1, -1]^T');
title('Different initial states produce distinguishable outputs');

% Optional Control System Toolbox check.
if exist('obsv', 'file') == 2
    fprintf('\nControl System Toolbox obsv(A1,C1):\n');
    disp(obsv(A1,C1));
end

% Optional Simulink model creation if Simulink is installed.
if exist('simulink', 'file') == 4
    modelName = 'Chapter13_Lesson1_Simulink';
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Continuous/State-Space', [modelName '/State-Space']);
    set_param([modelName '/State-Space'], 'A', 'A1', 'B', '[0;0]', 'C', 'C1', 'D', '0', 'X0', 'x01');

    add_block('simulink/Sinks/Scope', [modelName '/Scope']);
    add_line(modelName, 'State-Space/1', 'Scope/1');

    save_system(modelName);
    fprintf('Created optional Simulink model: %s.slx\n', modelName);
end

function O = local_observability_matrix(A, C)
    n = size(A, 1);
    O = [];
    Ak = eye(n);
    for k = 1:n
        O = [O; C*Ak]; %#ok<AGROW>
        Ak = Ak*A;
    end
end
