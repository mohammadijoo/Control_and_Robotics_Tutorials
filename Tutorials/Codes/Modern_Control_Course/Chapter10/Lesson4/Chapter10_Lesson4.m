% Chapter10_Lesson4.m
%
% Physical Interpretation: Actuator Placement and Authority
% Modern Control - Chapter 10, Lesson 4
%
% This script compares actuator placements for a two-degree-of-freedom
% mass-spring-damper system using MATLAB Control System Toolbox functions
% and a finite-horizon controllability Gramian.
%
% Related Simulink idea:
%   Use a State-Space block with A, B, C = eye(4), D = zeros(4,m).
%   Feed candidate inputs u(t) through Step, Signal Builder, or From Workspace.
%   Compare state trajectories for different B matrices.

clear; clc;

K = [2 -1; -1 2];
Damp = [0.08 0; 0 0.08];
M = eye(2);

A = [zeros(2) eye(2);
     -M\K     -M\Damp];

placements = {
    'force on mass 1 only', [1; 0];
    'force on mass 2 only', [0; 1];
    'same force on both masses', [1; 1];
    'independent forces on both masses', eye(2)
};

targetNames = {'displace mass 1', 'displace mass 2', 'relative motion', 'common motion'};
targets = [1 0 0 0;
           0 1 0 0;
           1/sqrt(2) -1/sqrt(2) 0 0;
           1/sqrt(2)  1/sqrt(2) 0 0];

T = 5;

for idx = 1:size(placements,1)
    name = placements{idx,1};
    G = placements{idx,2};
    B = [zeros(2,size(G,2)); M\G];

    Ctrb = ctrb(A,B);
    r = rank(Ctrb);

    W = integral(@(tau) expm(A*tau)*B*B'*expm(A'*tau), ...
                 0, T, 'ArrayValued', true);

    fprintf('\n============================================\n');
    fprintf('Actuator placement: %s\n', name);
    fprintf('Rank of ctrb(A,B): %d out of %d\n', r, size(A,1));
    fprintf('Eigenvalues of Wc(%g):\n', T);
    disp(eig(W).');

    fprintf('Directional authority v''*W*v:\n');
    for k = 1:size(targets,1)
        v = targets(k,:).';
        fprintf('  %-18s : %.8f\n', targetNames{k}, v.'*W*v);
    end

    [V,Lambda] = eig(A);
    modalInput = inv(V)*B;
    modalScores = vecnorm(modalInput,2,2);
    fprintf('Modal authority scores ||w_i''*B||_2:\n');
    for k = 1:length(modalScores)
        fprintf('  lambda = %.4f%+.4fi, score = %.8f\n', ...
            real(Lambda(k,k)), imag(Lambda(k,k)), modalScores(k));
    end
end
