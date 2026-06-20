% Chapter16_Lesson3.m
%
% Properties of Controllable Canonical Form (CCF) for analysis and design.
%
% This script constructs a SISO controllable canonical form realization,
% computes its controllability matrix, and performs pole placement by
% coefficient matching.
%
% Related MATLAB/Simulink tools:
%   Control System Toolbox: ss, tf, ctrb, place, acker
%   Simulink: State-Space block, Transfer Fcn block

clear; clc;

% D(s) = s^4 + 6 s^3 + 11 s^2 + 6 s + 2
% N(s) = 3 s^2 + 2 s + 1
% a = [a0 a1 ... a_{n-1}], b = [b0 b1 ... b_{n-1}]
a = [2 6 11 6];
b = [1 2 3 0];
n = numel(a);

A = zeros(n,n);
A(1:n-1,2:n) = eye(n-1);
A(n,:) = -a;

B = zeros(n,1);
B(n) = 1;

C = b;
D = 0;

% Manual controllability matrix [B AB ... A^(n-1)B]
Wc = zeros(n,n);
Apow = eye(n);
for k = 1:n
    Wc(:,k) = Apow * B;
    Apow = A * Apow;
end

disp('A ='); disp(A);
disp('B ='); disp(B);
disp('C ='); disp(C);
disp('Wc ='); disp(Wc);
fprintf('rank(Wc) = %d\n', rank(Wc));
fprintf('det(Wc)  = %.0f\n', det(Wc));

% Desired closed-loop poles.
desired_poles = [-2 -3 -4 -5];

% poly returns [1 alpha_{n-1} ... alpha0].
p_des = poly(desired_poles);
alpha = fliplr(p_des(2:end));   % [alpha0 ... alpha_{n-1}]

% CCF coefficient matching for u = -Kx + r:
K = alpha - a;
Acl = A - B*K;

disp('alpha = [alpha0 ... alpha_{n-1}]'); disp(alpha);
disp('K for u = -Kx + r'); disp(K);
disp('eig(A-BK) ='); disp(eig(Acl).');

% Transfer-function check, if Control System Toolbox is available:
% sys = ss(A,B,C,D);
% tf(sys)
% K_place = place(A,B,desired_poles)
% K_acker = acker(A,B,desired_poles)

% Simulink implementation note:
% Use a State-Space block with matrices A, B, C, D above.
% Feed back yK = K*x through a Sum block implementing u = r - K*x.
