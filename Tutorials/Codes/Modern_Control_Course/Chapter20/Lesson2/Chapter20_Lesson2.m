% Chapter20_Lesson2.m
%
% Exact minimal-realization reduction for Chapter 20, Lesson 2.
%
% Required toolbox for minreal/ss/tf:
%     Control System Toolbox
%
% Run:
%     Chapter20_Lesson2

clear; clc;

% Nonminimal example:
% x1: reachable and observable
% x2: unreachable but visible from initial condition
% x3: reachable but unobservable
A = diag([-1 -2 -3]);
B = [1; 0; 1];
C = [1 1 0];
D = 0;

n = size(A,1);
Wc = ctrb(A,B);
Wo = obsv(A,C);

fprintf('rank(Wc) = %d out of n = %d\n', rank(Wc), n);
fprintf('rank(Wo) = %d out of n = %d\n\n', rank(Wo), n);

sys = ss(A,B,C,D);
disp('Original transfer function:')
tf(sys)

% MATLAB performs cancellation of unreachable/unobservable dynamics in minreal.
sys_min = minreal(sys, 1e-8);

disp('Minimal realization produced by minreal:')
sys_min

disp('Minimal transfer function:')
tf(sys_min)

% Manual pedagogical result for this example.
Am = -1;
Bm = 1;
Cm = 1;
Dm = 0;

disp('Manual minimal realization:')
disp('Am = '); disp(Am);
disp('Bm = '); disp(Bm);
disp('Cm = '); disp(Cm);
disp('Dm = '); disp(Dm);

% Frequency-domain equality check.
samples = [0, 1, 2 + 1i];
for k = 1:length(samples)
    s = samples(k);
    Gfull = C*((s*eye(3)-A)\B) + D;
    Gmin  = Cm*((s-Am)\Bm) + Dm;
    fprintf('s = %-10s | Gfull = %-16s | Gmin = %-16s\n', ...
        num2str(s), num2str(Gfull), num2str(Gmin));
end
