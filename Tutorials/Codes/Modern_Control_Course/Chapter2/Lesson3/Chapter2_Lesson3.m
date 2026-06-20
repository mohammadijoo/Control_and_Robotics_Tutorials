% ===== Code block 1 extracted from Chapter2/Lesson3.html =====
% Example matrix
A = [4 1 0;
     1 3 1;
     0 1 2];

% 1) General eigen-decomposition
[V,D] = eig(A);
disp('Eigenvalues (diag(D)) = '); disp(diag(D));
disp('Check ||A - VDV^{-1}||_F = '); disp(norm(A - V*D/V, 'fro'));

% 2) Symmetric case: orthogonal diagonalization (A is symmetric here)
[Q,Lambda] = eig((A + A')/2);
disp('Check ||A - QΛQ^T||_F = '); disp(norm(A - Q*Lambda*Q', 'fro'));

% 3) Power iteration from scratch
x = randn(size(A,1),1);
x = x / norm(x);

lam_old = 0;
tol = 1e-10;
maxIter = 2000;

for k = 1:maxIter
    y = A*x;
    x = y / norm(y);
    lam = x' * A * x; % Rayleigh estimate

    if abs(lam - lam_old) < tol
        fprintf('Converged in %d iterations\\n', k);
        fprintf('lambda_hat = %.12f\\n', lam);
        disp('v_hat = '); disp(x);
        break;
    end
    lam_old = lam;
end

% ===== Code block 2 extracted from Chapter2/Lesson3.html =====
% MATLAB Function block code (input: A, output: lam)
function lam = fcn(A)
%#codegen
lam = eig(A);
end
