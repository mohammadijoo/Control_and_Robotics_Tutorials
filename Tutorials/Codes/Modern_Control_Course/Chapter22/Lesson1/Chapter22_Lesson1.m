% Chapter22_Lesson1.m
% Structure of state-feedback law: u = -K x + r
% Requires MATLAB. Control System Toolbox and Simulink sections are optional.

clear; clc; close all;

% Mass-spring-damper model:
% x1 = position, x2 = velocity, u = force
A = [0 1; -2 -0.4];
B = [0; 1];
C = [1 0];
D = 0;

% State-feedback gain. The structure is u = -K*x + r.
K = [4 2.6];
Acl = A - B*K;

disp('A - B*K =');
disp(Acl);
disp('Closed-loop eigenvalues:');
disp(eig(Acl));

% Constant command injected at the plant input
r = 1.0;
xbar = -Acl\(B*r);
ybar = C*xbar;
disp('Constant-r equilibrium xbar:');
disp(xbar);
disp('Constant-r equilibrium ybar:');
disp(ybar);

% From-scratch simulation with ode45
x0 = [0.2; 0.0];
f = @(t,x) A*x + B*(-K*x + r);
[t,x] = ode45(f, [0 8], x0);
u = zeros(length(t),1);
y = zeros(length(t),1);
for i = 1:length(t)
    u(i) = -K*x(i,:)' + r;
    y(i) = C*x(i,:)';
end

figure;
plot(t,y,'LineWidth',1.5); hold on;
plot(t,u,'LineWidth',1.5);
grid on;
xlabel('time [s]');
legend('y = Cx','u = -Kx + r');
title('Chapter22 Lesson1: State-feedback structure');

% Optional Control System Toolbox view:
% Closed-loop model from r to y is xdot = (A-BK)x + B r, y = Cx.
sys_cl = ss(Acl, B, C, D);
figure;
step(sys_cl, 8);
grid on;
title('Closed-loop response from r to y');

% Optional Simulink construction sketch.
% This creates the main blocks; depending on MATLAB release, block positions
% or State-Space parameter formatting may need minor manual adjustment.
%
% model = 'Chapter22_Lesson1_Simulink';
% new_system(model); open_system(model);
% add_block('simulink/Sources/Constant', [model '/r']);
% set_param([model '/r'], 'Value', '1');
% add_block('simulink/Continuous/State-Space', [model '/Plant']);
% set_param([model '/Plant'], 'A', mat2str(A), 'B', mat2str(B), ...
%     'C', mat2str(eye(2)), 'D', mat2str(zeros(2,1)));
% add_block('simulink/Math Operations/Gain', [model '/-K']);
% set_param([model '/-K'], 'Gain', mat2str(-K));
% add_block('simulink/Math Operations/Sum', [model '/Sum']);
% set_param([model '/Sum'], 'Inputs', '++');
% add_block('simulink/Sinks/Scope', [model '/Scope']);
% % Connect: Plant states -> -K -> Sum, r -> Sum, Sum -> Plant input.
% % save_system(model);
