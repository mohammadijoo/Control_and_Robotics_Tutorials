% Chapter27_Lesson2.m
% Feedforward gain design for reference tracking.

clear; clc; close all;

% Plant: xdot = A*x + B*u, y = C*x + D*u
A = [0 1; 0 0];
B = [0; 1];
C = [1 0];
D = 0;

% State feedback with desired poles at -2 and -2.
K = place(A, B, [-2 -2]);
Acl = A - B*K;
Ccl = C - D*K;

% Method 1: closed-loop DC-gain inversion.
G0 = D - Ccl*(Acl\B);
N_dc = inv(G0);

% Method 2: regulator equations.
n = size(A,1);
p = size(C,1);
Rosenbrock0 = [A B; C D];
RHS = [zeros(n,p); eye(p)];
Sol = Rosenbrock0 \ RHS;
X = Sol(1:n,:);
U = Sol(n+1:end,:);
N_reg = U + K*X;

fprintf('K =\n'); disp(K);
fprintf('N from DC gain =\n'); disp(N_dc);
fprintf('N from regulator equations =\n'); disp(N_reg);

% Closed-loop reference-to-output model.
Bcl = B*N_dc;
Dcl = D*N_dc;
sys_cl = ss(Acl, Bcl, Ccl, Dcl);
figure;
step(sys_cl, 6);
grid on;
title('Unit-step reference tracking with static feedforward gain');

% Simulink guidance:
% 1. Add a State-Space block with A, B, C, D.
% 2. Add a Gain block K in the negative feedback path from x to u.
% 3. Add a Gain block N_dc in the reference path.
% 4. Sum u = -K*x + N_dc*r and connect it to the plant input.
% 5. Scope y and verify that y approaches r for constant references.
