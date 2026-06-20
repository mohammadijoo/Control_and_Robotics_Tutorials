% Chapter24_Lesson3.m
% State transformations for MIMO pole placement in MATLAB/Octave.

clear; clc;

A_bar = [
    0 1 0 0;
    0 0 0 0;
    0 0 0 1;
    0 0 0 0
];
B_bar = [
    0 0;
    1 0;
    0 0;
    0 1
];

T = [
    1.0 0.2 0.0 0.1;
    0.1 1.0 0.2 0.0;
    0.0 0.1 1.0 0.3;
    0.2 0.0 0.1 1.0
];

A = T*A_bar/T;
B = T*B_bar;

% Transformed-coordinate feedback for poles {-2,-3} and {-4,-5}.
F = [
    6 5 0 0;
    0 0 20 9
];

K = F/T;
Acl = A - B*K;
Abar_cl = A_bar - B_bar*F;

fprintf('K = F/T:\n'); disp(K);
fprintf('eig(A - B*K):\n'); disp(eig(Acl));
fprintf('eig(A_bar - B_bar*F):\n'); disp(eig(Abar_cl));
fprintf('Similarity residual Frobenius norm = %.3e\n', norm(T\Acl*T - Abar_cl, 'fro'));

% Alternative direct MIMO pole placement if Control System Toolbox is available:
% desired_poles = [-2 -3 -4 -5];
% K_place = place(A, B, desired_poles);
