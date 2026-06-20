% Chapter19_Lesson1.m
% Modern Control - Chapter 19, Lesson 1
% Controllable/Uncontrollable Subspaces.
%
% Related MATLAB tools:
%   ctrb(A,B)          - controllability matrix
%   rank(Wc)           - numerical rank
%   orth(Wc)           - orthonormal basis for column space
%   null(Qc')          - orthogonal complement
%   ss(A,B,C,D)        - state-space model object
%   canon(sys,'modal') - canonical-form utility, when applicable

clear; clc;

A = [0 1 0;
     0 0 0;
     0 0 -2];
B = [0; 1; 0];

n = size(A,1);

% Use Control System Toolbox if available; otherwise build manually.
if exist('ctrb','file') == 2
    Wc = ctrb(A,B);
else
    Wc = [];
    Ak = eye(n);
    for k = 1:n
        Wc = [Wc, Ak*B]; %#ok<AGROW>
        Ak = Ak*A;
    end
end

r = rank(Wc);
fprintf('Rank(Wc) = %d out of n = %d\n', r, n);
if r == n
    fprintf('System is controllable.\n');
else
    fprintf('System is not controllable.\n');
end

Qc = orth(Wc);       % basis for controllable subspace
Qu = null(Qc');      % one orthogonal complement
T  = [Qc Qu];        % orthogonal coordinate transform when [Qc Qu] is square

Abar = T' * A * T;
Bbar = T' * B;

disp('Wc = [B AB ... A^(n-1)B]'); disp(Wc);
disp('Qc: basis for controllable subspace'); disp(Qc);
disp('Qu: orthogonal complement basis'); disp(Qu);
disp('Abar = T'' A T'); disp(Abar);
disp('Bbar = T'' B'); disp(Bbar);

if r < n
    disp('Lower-left block of Abar (should be numerically zero):');
    disp(Abar(r+1:end,1:r));
    disp('Lower block of Bbar (should be numerically zero):');
    disp(Bbar(r+1:end,:));
end

% Optional visualization of subspace dimension for this example
figure;
bar([r, n-r]);
set(gca,'XTickLabel',{'controllable','uncontrollable complement'});
ylabel('dimension');
title('Reachable Subspace Dimension Decomposition');
grid on;
