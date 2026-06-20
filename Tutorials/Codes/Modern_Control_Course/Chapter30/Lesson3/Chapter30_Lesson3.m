% Chapter30_Lesson3.m
% MATLAB/Simulink workflow for state-space analysis and design.
% Toolboxes commonly used: Control System Toolbox, Robust Control Toolbox,
% Simulink Control Design, Symbolic Math Toolbox, Model Predictive Control Toolbox.

clear; clc; close all;

% Mass-spring-damper plant: x1 = position, x2 = velocity
m = 1.0; c = 0.35; k = 2.0;
A = [0 1; -k/m -c/m];
B = [0; 1/m];
C = [1 0];
D = 0;
sys = ss(A, B, C, D);

fprintf('Open-loop poles:\n'); disp(eig(A));

Co = ctrb(A, B);
Ob = obsv(A, C);
fprintf('rank(ctrb) = %d, cond(ctrb) = %.3e\n', rank(Co), cond(Co));
fprintf('rank(obsv) = %d, cond(obsv) = %.3e\n', rank(Ob), cond(Ob));

% Pole placement design
p_des = [-2+1.5i, -2-1.5i];
K_place = place(A, B, p_des);
Acl_place = A - B*K_place;
fprintf('K_place =\n'); disp(K_place);
fprintf('Closed-loop poles by place:\n'); disp(eig(Acl_place));

% Continuous-time LQR design
Q = diag([20, 2]);
R = 0.5;
K_lqr = lqr(A, B, Q, R);
Acl_lqr = A - B*K_lqr;
fprintf('K_lqr =\n'); disp(K_lqr);
fprintf('Closed-loop poles by LQR:\n'); disp(eig(Acl_lqr));

% Gramian analysis
Wc = gram(sys, 'c');
Wo = gram(sys, 'o');
fprintf('Controllability Gramian:\n'); disp(Wc);
fprintf('Observability Gramian:\n'); disp(Wo);

% Closed-loop simulation from initial condition
sys_cl = ss(Acl_lqr, B, C, D);
t = linspace(0, 8, 600);
x0 = [1; 0];
[y, t, x] = initial(sys_cl, x0, t);
figure;
plot(t, x(:,1), 'LineWidth', 1.5); hold on;
plot(t, x(:,2), 'LineWidth', 1.5);
grid on; xlabel('time [s]'); ylabel('state');
legend('position', 'velocity');
title('Chapter 30 Lesson 3: LQR closed-loop response');

% Simulink note:
% 1. Insert a State-Space block with matrices A, B, C, D.
% 2. Add a Gain block with value -K_lqr in feedback from state output.
% 3. If the physical model is built from blocks, use linearize or linmod to obtain ss(A,B,C,D).
% 4. Compare Simulink output against initial(sys_cl,x0,t) for verification.
