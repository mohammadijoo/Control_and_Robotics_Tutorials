% Chapter18_Lesson1.m
% Jordan blocks and generalized eigenvectors for a state-space matrix.
% Toolboxes:
%   Symbolic Math Toolbox is useful for exact Jordan forms.
%   Control System Toolbox uses the same A-matrix in ss(A,B,C,D) models.

clear; clc;

A = [2 1 0 0;
     0 2 1 0;
     0 0 2 0;
     0 0 0 -1];

lambda = 2;
N = A - lambda * eye(4);

fprintf('Growth of generalized eigenspaces Ker((A-lambda I)^k):\n');
for k = 1:4
    Nk = N^k;
    fprintf('k=%d: rank=%d, nullity=%d\n', k, rank(Nk), size(A,1)-rank(Nk));
end

% Jordan chain:
% N*v1 = 0, N*v2 = v1, N*v3 = v2.
v3 = [0;0;1;0];
v2 = N*v3;
v1 = N*v2;

disp('Jordan chain for lambda=2:');
disp('v1 ='); disp(v1);
disp('v2 ='); disp(v2);
disp('v3 ='); disp(v3);

disp('Chain verification:');
disp('N*v1 ='); disp(N*v1);
disp('N*v2 ='); disp(N*v2);
disp('N*v3 ='); disp(N*v3);

w = [0;0;0;1];
T = [v1 v2 v3 w];
J = inv(T)*A*T;

disp('J = inv(T)*A*T:');
disp(J);

% Symbolic exponential of the size-3 Jordan block.
syms t
S = [0 1 0; 0 0 1; 0 0 0];
ExpJ3 = exp(lambda*t) * (eye(3) + t*S + (t^2/2)*S^2);
disp('exp(J3*t) =');
disp(ExpJ3);

% Simulink note:
% A state-space model with this internal matrix can be inserted using
% a State-Space block with A = A, B = chosen input matrix, C = chosen output
% matrix, and D = feedthrough matrix. The repeated pole at lambda=2 appears
% as a chain of integrator-like modal coordinates after x = T*z.
