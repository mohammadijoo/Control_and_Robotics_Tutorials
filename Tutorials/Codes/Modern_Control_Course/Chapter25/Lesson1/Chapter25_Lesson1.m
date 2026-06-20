% Chapter25_Lesson1.m
% Uncontrollable Modes and Unassignable Poles
%
% Requires for ctrb/place:
%   MATLAB Control System Toolbox
% If the toolbox is unavailable, the script still demonstrates the main result
% using explicit controllability matrices and eigenvalues.

clear; clc;

A = [0 0;
     0 2];
B = [1;
     0];

fprintf('A =\n'); disp(A);
fprintf('B =\n'); disp(B);

% Kalman controllability matrix from scratch
C = [B A*B];
fprintf('Kalman controllability matrix C = [B AB] =\n'); disp(C);
fprintf('rank(C) = %d out of n = %d\n', rank(C), size(A,1));

% Compare with Control System Toolbox if available
if exist('ctrb','file') == 2
    C_toolbox = ctrb(A,B);
    fprintf('ctrb(A,B) =\n'); disp(C_toolbox);
end

% PBH test at each eigenvalue
lambda = eig(A);
fprintf('\nPBH test:\n');
for i = 1:length(lambda)
    M = [lambda(i)*eye(size(A)) - A, B];
    r = rank(M);
    fprintf('  lambda = %.4g, rank([lambda I - A, B]) = %d/%d', lambda(i), r, size(A,1));
    if r < size(A,1)
        fprintf('  <-- uncontrollable mode');
    end
    fprintf('\n');
end

% State feedback: only one closed-loop pole can move.
fprintf('\nClosed-loop poles for several gains K = [k1 k2]:\n');
Ks = [0 0;
      3 0;
      8 100;
     -1 -50];
for i = 1:size(Ks,1)
    K = Ks(i,:);
    Acl = A - B*K;
    fprintf('  K = [%8.3f %8.3f] -> eig(A-BK) = ', K(1), K(2));
    disp(eig(Acl).');
end

% Full pole placement is not valid because the pair is not controllable.
fprintf('\nAttempting place(A,B,[-4 -5]):\n');
if exist('place','file') == 2
    try
        Kplace = place(A,B,[-4 -5]);
        disp(Kplace);
    catch ME
        fprintf('Pole placement failed: %s\n', ME.message);
    end
else
    fprintf('Control System Toolbox place() was not found.\n');
end

% A controllable comparison system
Ac = [0 1;
     -2 -3];
Bc = [0; 1];
fprintf('\nControllable comparison system:\n');
fprintf('rank([Bc Ac*Bc]) = %d\n', rank([Bc Ac*Bc]));
if exist('place','file') == 2
    Kc = place(Ac,Bc,[-4 -5]);
    fprintf('Kc from place = '); disp(Kc);
    fprintf('eig(Ac-Bc*Kc) = '); disp(eig(Ac-Bc*Kc).');
end
