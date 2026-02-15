% Inertia-like symmetric matrix
I = [0.20 0.01 0.00;
     0.01 0.15 0.00;
     0.00 0.00 0.10];

% Eigen-decomposition
[V, D] = eig(I);   % I * V = V * D, columns of V are eigenvectors

lambda = diag(D);

disp('Eigenvalues (principal moments):');
disp(lambda);

disp('Eigenvectors (principal axes as columns):');
disp(V);

% Check orthogonality (spectral theorem)
disp('V''*V (should be identity for symmetric I):');
disp(V' * V);

% Example: transform angular velocity omega into principal coordinates
omega_body = [1; 2; 0.5];   % rad/s in body frame
omega_principal = V' * omega_body;
kinetic_energy = 0.5 * omega_principal' * D * omega_principal;
fprintf('Rotational kinetic energy: %f J\n', kinetic_energy);
      
