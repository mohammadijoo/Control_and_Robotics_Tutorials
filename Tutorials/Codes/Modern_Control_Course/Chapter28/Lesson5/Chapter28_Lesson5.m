% Chapter28_Lesson5.m
%
% Preview of optimal state-feedback using continuous-time LQR.
% Requires Control System Toolbox for lqr/care. The optional Simulink part
% creates a closed-loop State-Space block if Simulink is available.

clear; clc; close all;

A = [0 1; 0 0];
B = [0; 1];
Q = diag([10 1]);
R = 0.2;

[K, P, closedLoopPoles] = lqr(A, B, Q, R);
Acl = A - B*K;

fprintf('P =\n');
disp(P);
fprintf('K =\n');
disp(K);
fprintf('closed-loop poles =\n');
disp(closedLoopPoles);

x0 = [1; 0];
tf = 8;
[t, x] = ode45(@(t, x) Acl*x, [0 tf], x0);
u = -(K*x')';

runningCost = sum((x*Q).*x, 2) + (u.^2)*R;
J_est = trapz(t, runningCost);

fprintf('finite-horizon estimated cost over [0, 8] = %.8f\n', J_est);
fprintf('infinite-horizon value x0''*P*x0 = %.8f\n', x0'*P*x0);

figure;
plot(t, x(:,1), 'LineWidth', 1.5); hold on;
plot(t, x(:,2), 'LineWidth', 1.5);
plot(t, u, 'LineWidth', 1.5);
grid on;
xlabel('time');
legend('position', 'velocity', 'control');
title('Chapter 28 Lesson 5: LQR closed-loop response');

% Optional Simulink generation: closed-loop model x_dot = (A-BK)x.
if exist('new_system', 'file') == 2
    mdl = 'Chapter28_Lesson5_Simulink';
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    new_system(mdl);
    open_system(mdl);

    add_block('simulink/Continuous/State-Space', [mdl '/ClosedLoopStateSpace'], ...
        'A', mat2str(Acl), ...
        'B', mat2str(zeros(2,1)), ...
        'C', mat2str(eye(2)), ...
        'D', mat2str(zeros(2,1)), ...
        'X0', mat2str(x0), ...
        'Position', [150 100 330 170]);

    add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
        'Position', [430 105 500 165]);

    add_line(mdl, 'ClosedLoopStateSpace/1', 'Scope/1');
    set_param(mdl, 'StopTime', num2str(tf));
    save_system(mdl);
    fprintf('Created optional Simulink model: %s.slx\n', mdl);
end
