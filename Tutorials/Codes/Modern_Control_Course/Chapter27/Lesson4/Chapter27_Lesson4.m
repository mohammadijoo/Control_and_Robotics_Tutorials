% Chapter27_Lesson4.m
% State-space disturbance rejection using measured feedforward and integral action.
% Required toolbox for place(): Control System Toolbox.

clear; clc; close all;

A = [0 1; -2 -0.6];
B = [0; 1];
E = [0; 1];
C = [1 0];
d0 = 0.5;
x0 = [0.4; 0];

%% 1) State feedback only: u = -Kx
K = place(A, B, [-2 -3]);
f1 = @(t, x) A*x + B*(-K*x) + E*d0;
[t1, x1] = ode45(f1, [0 10], x0);
y1 = (C*x1')';

%% 2) Measured disturbance feedforward from regulator equations
M = [A - B*K, B; C, 0];
rhs = [-E; 0];
sol = M \ rhs;
Pi = sol(1:2);
Gamma = sol(3);

f2 = @(t, x) A*x + B*(-K*x + Gamma*d0) + E*d0;
[t2, x2] = ode45(f2, [0 10], x0);
y2 = (C*x2')';

%% 3) Unmeasured constant disturbance rejection with xi_dot = y
Aaug = [A, zeros(2,1); C, 0];
Baug = [B; 0];
Kaug = place(Aaug, Baug, [-2 -2.5 -3]);
Kx = Kaug(1:2);
Ki = Kaug(3);

f3 = @(t, xa) [A*xa(1:2) + B*(-Kx*xa(1:2) - Ki*xa(3)) + E*d0;
               C*xa(1:2)];
[t3, xa3] = ode45(f3, [0 10], [x0; 0]);
y3 = (C*xa3(:,1:2)')';

fprintf('K = [%g %g]\n', K(1), K(2));
fprintf('Measured disturbance Gamma = %g\n', Gamma);
fprintf('Kaug = [%g %g %g]\n', Kaug(1), Kaug(2), Kaug(3));
fprintf('Final outputs: feedback only=%g, feedforward=%g, integral=%g\n', y1(end), y2(end), y3(end));

figure;
plot(t1, y1, 'LineWidth', 1.2); hold on;
plot(t2, y2, 'LineWidth', 1.2);
plot(t3, y3, 'LineWidth', 1.2);
yline(0, '--'); grid on;
xlabel('time [s]'); ylabel('controlled output y');
title('State-space disturbance rejection');
legend('state feedback only', 'measured feedforward', 'integral action', 'Location', 'best');

%% Optional Simulink model: augmented closed-loop from disturbance to y
% The model is: d0 -> State-Space(Acl, Bd, Cy, Dy) -> Scope.
if exist('simulink', 'file') == 2
    model = 'Chapter27_Lesson4_Simulink_Model';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);

    Acl = [A - B*Kx, -B*Ki; C, 0];
    Bd = [E; 0];
    Cy = [C, 0];
    Dy = 0;

    add_block('simulink/Sources/Constant', [model '/Constant disturbance'], ...
        'Value', num2str(d0), 'Position', [40 80 130 110]);
    add_block('simulink/Continuous/State-Space', [model '/Closed-loop augmented plant'], ...
        'A', mat2str(Acl), 'B', mat2str(Bd), 'C', mat2str(Cy), 'D', mat2str(Dy), ...
        'Position', [190 60 370 130]);
    add_block('simulink/Sinks/Scope', [model '/Output y'], ...
        'Position', [430 72 500 118]);
    add_line(model, 'Constant disturbance/1', 'Closed-loop augmented plant/1');
    add_line(model, 'Closed-loop augmented plant/1', 'Output y/1');
    set_param(model, 'StopTime', '10');
    save_system(model);
    fprintf('Simulink model saved as %s.slx\n', model);
end
