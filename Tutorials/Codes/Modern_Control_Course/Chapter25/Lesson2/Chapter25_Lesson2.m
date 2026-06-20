% Chapter25_Lesson2.m
% Role of transmission zeros in state-feedback design limits.
%
% Required toolbox:
%   Control System Toolbox
%
% This script uses
%       G(s) = (1 - s)/(s^2 + 3s + 2)
% and demonstrates that static state feedback moves closed-loop poles but
% leaves the transmission zero fixed.

clear; clc; close all;

A = [0 1; -2 -3];
B = [0; 1];
C = [1 -1];
D = 0;

sys = ss(A, B, C, D);

disp('Open-loop transfer function:');
G = tf(sys)

disp('Open-loop transmission zeros:');
z_open = tzero(sys)

disp('Open-loop poles:');
p_open = pole(sys)

% Place closed-loop poles at -5 and -6.
desired_poles = [-5 -6];
K = place(A, B, desired_poles);
Acl = A - B*K;
sys_cl = ss(Acl, B, C, D);

disp('State-feedback gain K:');
disp(K);

disp('Closed-loop transfer function from v to y:');
Gcl = tf(sys_cl)

disp('Closed-loop transmission zeros:');
z_closed = tzero(sys_cl)

disp('Closed-loop poles:');
p_closed = pole(sys_cl)

% Rosenbrock rank test at s = +1.
s = 1;
P = [s*eye(size(A)) - A, -B; C, D];
Pcl = [s*eye(size(Acl)) - Acl, -B; C, D];

disp('Rank of Rosenbrock matrix at s = +1:');
disp(rank(P));
disp('Rank of closed-loop Rosenbrock matrix at s = +1:');
disp(rank(Pcl));

% Compare step responses. The RHP zero causes inverse-response behavior
% that is not removed by state feedback.
figure;
step(sys, sys_cl);
grid on;
legend('open-loop input-to-output', 'closed-loop v-to-output', 'Location', 'best');
title('Transmission zero remains fixed under state feedback');

% Simulink implementation note:
% 1. Add a State-Space block for (A,B,C,D).
% 2. Add a Gain block K in feedback from x to u if states are exposed.
% 3. Use u = -Kx + v.
% 4. Compare the Transfer Function/Linear Analysis zeros before and after K.
% The poles change, but the zero at s = +1 remains.
