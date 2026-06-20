% Chapter14_Lesson4.m
% Observability in canonical (observable) forms.
% Run in MATLAB or GNU Octave.

clear; clc;

% Example transfer function:
% G(s) = (2 s^2 + 3 s + 4)/(s^3 + 6 s^2 + 11 s + 6)
% den = [a0 a1 ... a_{n-1}], num = [b0 b1 ... b_{n-1}]
den = [6 11 6];
num = [4 3 2];

[Ao, Bo, Co] = observable_companion(den, num);
O = observability_matrix(Ao, Co);

fprintf('A_o =\n'); disp(Ao);
fprintf('B_o =\n'); disp(Bo);
fprintf('C_o =\n'); disp(Co);
fprintf('Observability matrix O =\n'); disp(O);
fprintf('rank(O) = %d out of %d\n', rank(O), size(Ao,1));
fprintf('det(O) = %.6g\n', det(O));

% MATLAB Control System Toolbox alternative:
% Obuiltin = obsv(Ao, Co);
% rank(Obuiltin)

% Reconstruct x(0) from output derivatives under zero input:
x0_true = [1; -2; 0.5];
eta = O*x0_true;              % eta = [y(0); ydot(0); yddot(0)]
x0_hat = O\eta;
fprintf('x0_true =\n'); disp(x0_true);
fprintf('x0_hat =\n'); disp(x0_hat);

function [A, B, C] = observable_companion(den, num)
    n = length(den);
    A = zeros(n,n);
    A(:,1) = -flipud(den(:));
    for i = 1:n-1
        A(i,i+1) = 1;
    end
    C = zeros(1,n);
    C(1) = 1;
    if nargin < 2
        B = zeros(n,1);
    else
        B = flipud(num(:));
    end
end

function O = observability_matrix(A, C)
    n = size(A,1);
    O = zeros(n,n);
    Ak = eye(n);
    for k = 1:n
        O(k,:) = C*Ak;
        Ak = Ak*A;
    end
end
