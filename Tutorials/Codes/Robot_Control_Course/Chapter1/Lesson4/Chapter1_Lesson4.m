
% Example: 2-DOF linearized joint error dynamics
A = [0 0 1 0;
     0 0 0 1;
    -50 0 -10 0;
     0 -40 0 -12];

% Choose Q > 0
Q = eye(4);

% Solve A' P + P A = -Q
P = lyap(A', Q);

% Check eigenvalues of A (Hurwitz condition)
lambda = eig(A);

disp('Eigenvalues of A:');
disp(lambda);
disp('Lyapunov matrix P:');
disp(P);

% Simulink:
%  - Create a State-Space block with A, B = zeros(4,1), C = eye(4), D = 0.
%  - Feed an initial condition x0 in an IC block.
%  - Scope the states to visualize convergence.
