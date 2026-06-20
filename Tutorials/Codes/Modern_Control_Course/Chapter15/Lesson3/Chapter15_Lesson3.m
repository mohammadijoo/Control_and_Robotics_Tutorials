% Chapter15_Lesson3.m
% Finite-horizon observability Gramian and qualitative sensor placement.
%
% Related MATLAB/Simulink tools:
%   Control System Toolbox: ss, obsv, gram, lyap
%   Simulink: State-Space block, To Workspace block, Linear Analysis Tool
%
% This script uses finite-horizon quadrature so that the teaching example
% does not depend on asymptotic stability.

clear; clc;

A = [ 0.0   1.0   0.0;
     -2.0  -0.45  0.8;
      0.0  -0.7  -1.25 ];

n = size(A,1);
T = 6.0;
N = 2000;
t = linspace(0,T,N);
dt = t(2)-t(1);

candidateNames = ["x1","x2","x3"];

fprintf("Single-sensor scores\n");
for i = 1:n
    C = zeros(1,n);
    C(i) = 1.0;
    W = finiteHorizonWo(A,C,t,dt);
    printMetrics(candidateNames(i), W);
end

fprintf("\nTwo-sensor scores\n");
bestScore = -Inf;
bestName = "";
for i = 1:n
    for j = i+1:n
        C = zeros(2,n);
        C(1,i) = 1.0;
        C(2,j) = 1.0;
        W = finiteHorizonWo(A,C,t,dt);
        score = regularizedLogDet(W,1e-8);
        name = candidateNames(i) + "," + candidateNames(j);
        printMetrics(name, W);
        if score > bestScore
            bestScore = score;
            bestName = name;
        end
    end
end
fprintf("\nBest qualitative two-sensor set by log-det: %s\n", bestName);

% Infinite-horizon alternative for asymptotically stable A:
% C = [1 0 0; 0 1 0];
% Wo_inf = lyap(A', C'*C);  % solves A'*Wo + Wo*A + C'*C = 0

% Simulink note:
% Build a model with a State-Space block using matrices A, B, C, D.
% For candidate sensors, change C or route selected states through a Selector
% block, log y(t) with To Workspace, and numerically integrate y(t)'*y(t)
% for several initial conditions to compare output energy.

function W = finiteHorizonWo(A,C,t,dt)
    n = size(A,1);
    W = zeros(n,n);
    Q = C'*C;
    for k = 1:length(t)
        E = expm(A*t(k));
        W = W + E'*Q*E*dt;
    end
    W = 0.5*(W + W');
end

function value = regularizedLogDet(W,epsVal)
    e = eig(W + epsVal*eye(size(W)));
    value = sum(log(max(real(e),epsVal)));
end

function printMetrics(name,W)
    e = eig(0.5*(W+W'));
    tr = trace(W);
    ld = regularizedLogDet(W,1e-8);
    lamMin = min(real(e));
    lamMax = max(real(e));
    if lamMin <= 1e-10
        condVal = Inf;
    else
        condVal = lamMax/lamMin;
    end
    fprintf("%8s  trace=%12.6f  logdet_eps=%12.6f  lambda_min=%12.4e  cond=%12.4e\n", ...
        name, tr, ld, lamMin, condVal);
end
