% Chapter14_Lesson5.m
% LTV observability Gramian and initial-state reconstruction.
% Simulink note:
%   Implement A(t), C(t), Phi_dot = A(t)Phi, and W_dot = Phi'C'C Phi
%   with MATLAB Function blocks and Integrator blocks. This script gives
%   the numerical reference trajectory for those blocks.

clear; clc;

t0 = 0;
tf = 6;
n = 2;
x0_true = [1; -0.7];

aug0 = [reshape(eye(n), n*n, 1); reshape(zeros(n), n*n, 1); zeros(n,1)];

opts = odeset('RelTol',1e-10,'AbsTol',1e-12);
[t, aug] = ode45(@augmented_rhs, [t0 tf], aug0, opts);

Phi_all = aug(:, 1:n*n);
W_all = aug(:, n*n+1:2*n*n);
z_all = aug(:, 2*n*n+1:end);

W = reshape(W_all(end,:), n, n);
z = z_all(end,:).';
x0_hat = W \ z;

disp('Observability Gramian W_o:');
disp(W);
disp('Eigenvalues of W_o:');
disp(eig(W).');
disp('Condition number:');
disp(cond(W));
disp('True x0:');
disp(x0_true.');
disp('Reconstructed x0:');
disp(x0_hat.');
disp('Reconstruction error norm:');
disp(norm(x0_hat - x0_true));

function d_aug = augmented_rhs(t, aug)
    n = 2;
    Phi = reshape(aug(1:n*n), n, n);

    At = A_of_t(t);
    Ct = C_of_t(t);

    dPhi = At * Phi;
    dW = Phi.' * Ct.' * Ct * Phi;

    y = Ct * Phi * [1; -0.7];  % noiseless homogeneous output
    dz = Phi.' * Ct.' * y;

    d_aug = [reshape(dPhi, n*n, 1); reshape(dW, n*n, 1); dz];
end

function At = A_of_t(t)
    At = [0, 1;
          -(2.0 + 0.5*sin(1.3*t)), -(0.15 + 0.05*cos(t))];
end

function Ct = C_of_t(t)
    Ct = [1.0, 0.2*sin(0.7*t)];
end
