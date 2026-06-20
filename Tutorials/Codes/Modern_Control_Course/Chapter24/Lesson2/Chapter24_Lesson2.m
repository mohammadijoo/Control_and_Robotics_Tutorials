% Chapter24_Lesson2.m
% Degrees of freedom and closed-loop eigenstructure for MIMO pole placement.
% Model: x_dot = A x + B u, u = -K x, so A_cl = A - B K.
%
% Toolboxes:
%   - No toolbox is required for the constructive eigenstructure part.
%   - Control System Toolbox can be used for place(A,B,poles) comparison.

clear; clc;

A = [0 1 0;
     0 0 1;
    -2 -3 -4];

B = [0 0;
     1 0;
     0 1];

desired_poles = [-1 -2 -5];
gammas = [0.2 -0.4 0.8];  % extra MIMO eigenvector-shaping degrees of freedom

V = zeros(3, 3);
Z = zeros(2, 3);

for j = 1:3
    lambda = desired_poles(j);
    gamma = gammas(j);

    v = [1;
         lambda;
         lambda^2 + gamma];

    z = [gamma;
         -2 - 3*lambda + (-4 - lambda)*(lambda^2 + gamma)];

    V(:, j) = v;
    Z(:, j) = z;
end

if abs(det(V)) < 1e-10
    error('Chosen eigenvectors are nearly singular. Change gammas.');
end

K = Z / V;             % same as Z * inv(V), but numerically preferable
Acl = A - B*K;
residual = A*V - B*Z - V*diag(desired_poles);

disp('A ='); disp(A);
disp('B ='); disp(B);
disp('V ='); disp(V);
disp('Z = K V ='); disp(Z);
disp('K ='); disp(K);
disp('eig(A - B*K) ='); disp(eig(Acl).');
disp('cond(V) ='); disp(cond(V));
disp('max eigenstructure residual ='); disp(max(abs(residual), [], 'all'));

% Optional comparison:
% K_place = place(A, B, desired_poles);
% disp('K from MATLAB place ='); disp(K_place);
% disp('eig(A - B*K_place) ='); disp(eig(A - B*K_place).');
