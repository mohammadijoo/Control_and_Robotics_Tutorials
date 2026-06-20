% Chapter12_Lesson3.m
% Gramian-based controllability tests in MATLAB/Simulink style.
% Requires Control System Toolbox for ctrb and gram; numerical integration works without gram.

clear; clc;

A = [0 1; -2 -3];
B = [0; 1];
C = eye(2);
D = zeros(2,1);
T = 2.0;

% Classical Kalman rank test for comparison
Co = ctrb(A,B);
fprintf('Rank of Kalman controllability matrix = %d\n', rank(Co));

% Finite-horizon controllability Gramian:
% Wc(T) = int_0^T expm(A*tau)*B*B'*expm(A'*tau) d tau
N = 2000;
tau = linspace(0,T,N+1);
h = T/N;
W = zeros(size(A));
for k = 1:(N+1)
    E = expm(A*tau(k));
    F = E*B*B'*E';
    if k == 1 || k == N+1
        coeff = 1;
    elseif mod(k-1,2) == 0
        coeff = 2;
    else
        coeff = 4;
    end
    W = W + coeff*F;
end
W = (h/3)*W;
W = 0.5*(W + W');

disp('Finite-horizon Wc(T):');
disp(W);
disp('Eigenvalues of Wc(T):');
disp(eig(W));
fprintf('det(Wc(T)) = %.12g\n', det(W));
fprintf('rank(Wc(T)) = %d\n', rank(W,1e-9));

% Infinite-horizon Gramian exists if A is Hurwitz.
if all(real(eig(A)) < 0)
    Winf = lyap(A, B*B');   % solves A*W + W*A' + B*B' = 0
    disp('Infinite-horizon Wc:');
    disp(Winf);
    disp('Eigenvalues of infinite-horizon Wc:');
    disp(eig(Winf));
end

% Minimum-energy steering from x(0)=0 to xf over [0,T].
xf = [1; 0];
eta = W\xf;
Emin = xf'*(W\xf);
fprintf('Minimum energy to reach xf = [1;0] over T=%.2f is %.12g\n', T, Emin);

sample_t = linspace(0,T,9);
fprintf('Sampled minimum-energy u*(t):\n');
for i = 1:length(sample_t)
    t = sample_t(i);
    u = B'*expm(A'*(T-t))*eta;
    fprintf('t = %.3f, u = %.12g\n', t, u);
end

% Simulink note:
% 1) Place a State-Space block with A, B, C = eye(2), D = zeros(2,1).
% 2) Feed the computed u*(t) through a From Workspace block.
% 3) Verify that the final simulated state approaches xf at t = T.
