% Chapter 13 - Vibrations and Multi-Degree-of-Freedom (MDOF) Systems
% Lesson 1: Natural Frequencies and Normal Modes (Eigenvalue Problems)
%
% This script:
%   1) Builds (M,K) for a 3-DOF undamped mass-spring chain.
%   2) Solves generalized symmetric eigenproblem: K*phi = (w^2)*M*phi.
%   3) Mass-normalizes Phi so that Phi'*M*Phi = I.
%   4) Verifies diagonalization: Phi'*K*Phi = diag(w^2).
%   5) Simulates free vibration using modal superposition.
%
% Simulink note:
%   You can implement the state-space model xdot = A x with x=[q; qdot] and
%   A = [0 I; -M\K 0], using a State-Space block. Natural frequencies are
%   abs(imag(eig(A))) for the undamped case.

clear; clc;

m = [2.0; 1.5; 1.0];
k = [200.0; 300.0; 250.0];

M = diag(m);
K = zeros(3,3);

% Assemble chain: wall-k1-m1-k2-m2-k3-m3-(free end)
for i=1:3
    K(i,i) = K(i,i) + k(i);
    if i > 1
        K(i,i)   = K(i,i)   + k(i-1);
        K(i,i-1) = K(i,i-1) - k(i-1);
        K(i-1,i) = K(i-1,i) - k(i-1);
    end
end

% Generalized eigenproblem
[Phi, D] = eig(K, M);     % columns of Phi are modes; D diag of w^2 (not necessarily sorted)
w2 = diag(D);

% Sort ascending
[w2s, idx] = sort(max(w2,0));
Phi = Phi(:, idx);
w = sqrt(w2s);

% Mass-normalize
for i=1:3
    mi = Phi(:,i)'*M*Phi(:,i);
    Phi(:,i) = Phi(:,i)/sqrt(mi);
end

disp('Natural frequencies (rad/s):');
disp(w.');

Mt = Phi'*M*Phi;
Kt = Phi'*K*Phi;

disp('Phi^T M Phi (should be I):');
disp(Mt);

disp('Phi^T K Phi (should be diag(w^2)):');
disp(Kt);

% Free response via modal superposition
t = linspace(0,5,1501);
q0 = [0.02; 0.0; -0.01];
v0 = [0.0; 0.0; 0.0];

eta0 = Phi'*M*q0;
etaDot0 = Phi'*M*v0;

eta = zeros(3, numel(t));
for i=1:3
    if w(i) < 1e-12
        eta(i,:) = eta0(i) + etaDot0(i)*t;
    else
        eta(i,:) = eta0(i)*cos(w(i)*t) + (etaDot0(i)/w(i))*sin(w(i)*t);
    end
end

q = Phi*eta;

figure;
plot(t, q(1,:), t, q(2,:), t, q(3,:));
grid on;
xlabel('t (s)');
ylabel('displacement (m)');
title('3-DOF undamped free vibration (modal superposition)');
legend('q1','q2','q3');
