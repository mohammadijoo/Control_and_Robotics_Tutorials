% Chapter24_Lesson5.m
% Modern Control - Chapter 24, Lesson 5
% Design examples for multi-input pole placement in MATLAB / Simulink.
%
% Required toolbox for place/ss/initial:
%   Control System Toolbox

clear; clc; close all;

A = [ 0  1  0  0;
      0  0  1  0;
      0  0  0  1;
     -2 -5 -4 -1 ];
B = [ 0 0;
      1 0;
      0 0;
      0 1 ];

p = [-1 -2 -3 -4];

Co = ctrb(A,B);
fprintf('rank(Co) = %d out of %d\n', rank(Co), size(A,1));

% MATLAB's robust MIMO pole placement.
K_place = place(A,B,p);
disp('K from place(A,B,p):');
disp(K_place);
disp('eig(A-B*K_place):');
disp(eig(A-B*K_place));

% From-scratch eigenstructure-assignment implementation.
[K_eig,V,F] = localEigenstructurePlacement(A,B,p);
disp('K from eigenstructure assignment:');
disp(K_eig);
disp('eig(A-B*K_eig):');
disp(eig(A-B*K_eig));
fprintf('cond(V) = %.3e\n', cond(V));

% Closed-loop simulation from an initial condition.
x0 = [1; -0.5; 0.8; 0];
sys_cl = ss(A-B*K_eig, zeros(4,2), eye(4), zeros(4,2));
t = 0:0.01:8;
initial(sys_cl, x0, t);
grid on;
title('Closed-loop response with multi-input pole placement');

% Simulink construction idea:
% 1) Add a State-Space block with A, B, C=eye(4), D=zeros(4,2).
% 2) Add a Gain block K_eig and set input u = -K_eig*x.
% 3) Feed the state output back through the negative gain into the input port.
% 4) Use Scope blocks for the state vector and control vector.

function [K,V,F] = localEigenstructurePlacement(A,B,p)
    n = size(A,1);
    m = size(B,2);
    V = zeros(n,n);
    F = zeros(m,n);
    rng(24);

    for i = 1:n
        lambda = p(i);
        M = [A - lambda*eye(n), -B];
        N = null(M);
        if isempty(N)
            error('Empty nullspace for lambda = %g', lambda);
        end
        accepted = false;
        for trial = 1:300
            if trial == 1
                q = ones(size(N,2),1);
            else
                q = randn(size(N,2),1);
            end
            s = N*q;
            v = real(s(1:n));
            f = real(s(n+1:n+m));
            Vtrial = V;
            Vtrial(:,i) = v;
            if rank(Vtrial(:,1:i)) == i
                V(:,i) = v;
                F(:,i) = f;
                accepted = true;
                break;
            end
        end
        if ~accepted
            error('Could not select an independent eigenvector.');
        end
    end
    K = F / V;
end
