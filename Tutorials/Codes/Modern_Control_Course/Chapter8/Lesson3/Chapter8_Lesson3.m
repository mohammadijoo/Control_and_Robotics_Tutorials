% Chapter8_Lesson3.m
%
% Computing the state transition matrix Phi(t) = expm(A*t) via eigen-decomposition.
%
% MATLAB functions used:
%   eig  - eigenvalues/eigenvectors
%   expm - robust built-in matrix exponential for comparison
%   cond - conditioning of eigenvector matrix
%
% Simulink note:
%   The helper function phi_via_eigendecomposition can be placed in a MATLAB
%   Function block for educational demonstrations. For real-time deployment,
%   precompute modal data or use validated numerical routines.

clear; clc;

A = [-1.0,  2.0,  0.0;
      0.0, -2.0,  0.0;
      0.0,  0.0, -0.5];

t = 2.0;

PhiEig = phi_via_eigendecomposition(A, t);
PhiExpm = expm(A * t);

disp('A =');
disp(A);

disp('Phi(t) via eigen-decomposition =');
disp(PhiEig);

disp('Phi(t) via expm(A*t) =');
disp(PhiExpm);

disp('Frobenius error =');
disp(norm(PhiEig - PhiExpm, 'fro'));

x0 = [1.0; -1.0; 2.0];
xt = PhiEig * x0;

disp('x(t) = Phi(t) x0 =');
disp(xt);

function Phi = phi_via_eigendecomposition(A, t)
    [V, D] = eig(A);

    if rank(V) < size(A, 1)
        error('A is not diagonalizable: eigenvector matrix V is rank deficient.');
    end

    c = cond(V);
    if c > 1e10
        warning('Eigenvector matrix is ill-conditioned: cond(V) = %g', c);
    end

    Phi = V * exp(D * t) / V;

    if isreal(A) && norm(imag(Phi), 'fro') < 1e-10
        Phi = real(Phi);
    end
end
