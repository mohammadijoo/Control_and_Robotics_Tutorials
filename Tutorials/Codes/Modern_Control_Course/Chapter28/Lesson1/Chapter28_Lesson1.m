% Chapter28_Lesson1.m
% Quadratic forms in state and input variables for Modern Control.
%
% Recommended MATLAB toolboxes:
%   Control System Toolbox for ss(), lsim(), lqr() in later optimal-control lessons.
%   Simulink for block-diagram implementation of x'Qx + u'Ru.
%
% This lesson intentionally uses a fixed stabilizing feedback gain K rather than
% deriving an optimal gain.

clear; clc;

A = [0 1; -2 -0.4];
B = [0; 1];

Q = diag([10 1]);      % state weighting matrix
R = 0.25;              % input weighting matrix
N = [0; 0.15];         % state-input cross weighting

x = [0.7; -0.2];
u = 0.4;

fprintf('x''Qx = %.6f\n', quadratic_form(x,Q));
fprintf('stage cost = %.6f\n', stage_cost(x,u,Q,R,N));

[RinvNT, Schur] = complete_square(Q,R,N);
disp('R^{-1}N^T ='); disp(RinvNT);
disp('Q - N R^{-1} N^T ='); disp(Schur);
disp('Eigenvalues of Q:'); disp(eig((Q+Q')/2));
disp('Eigenvalues of R:'); disp(eig((R+R')/2));
disp('Eigenvalues of Schur complement:'); disp(eig((Schur+Schur')/2));

K = [3 2];
x0 = [1; 0];
tf = 8;

z0 = [x0; 0];
[t,z] = ode45(@(t,z) augmented_rhs(t,z,A,B,K,Q,R), [0 tf], z0);
J = z(end,end);
fprintf('Finite-horizon quadratic performance J = %.6f\n', J);

% Optional Simulink skeleton:
% This creates a simple empty model and stores variables in the workspace.
% Students can add State-Space, Gain, Product, Sum, and Integrator blocks.
modelName = 'Chapter28_Lesson1_Simulink_Model';
if ~bdIsLoaded(modelName)
    new_system(modelName);
    open_system(modelName);
    assignin('base','A',A);
    assignin('base','B',B);
    assignin('base','Q',Q);
    assignin('base','R',R);
    assignin('base','K',K);
end

function value = quadratic_form(z,M)
    value = z' * M * z;
end

function value = stage_cost(x,u,Q,R,N)
    value = x' * Q * x + 2*x' * N * u + u' * R * u;
end

function [RinvNT, Schur] = complete_square(Q,R,N)
    RinvNT = R \ N';
    Schur = Q - N * RinvNT;
end

function dz = augmented_rhs(~,z,A,B,K,Q,R)
    n = size(A,1);
    x = z(1:n);
    u = -K*x;
    dx = A*x + B*u;
    dJ = x'*Q*x + u'*R*u;
    dz = [dx; dJ];
end
