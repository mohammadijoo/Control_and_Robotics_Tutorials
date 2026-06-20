% Define the same matrix A
A = [1 1 0;
     0 1 1];

disp('A =');
disp(A);

% Rank and null space
rA = rank(A);
disp(['rank(A) = ' num2str(rA)]);

N = null(A, 'r');  % 'r' requests a rational (exact) basis when possible
disp('Null-space basis (columns):');
disp(N);

% Verify that A*N is (numerically) zero
disp('A * N =');
disp(A * N);

% Example: solve a least-squares problem A*x ≈ b
b = [1; 2];
x_ls = A\b;   % MATLAB backslash chooses least-squares for overdetermined systems
disp('Least-squares solution x_ls =');
disp(x_ls);

% Simulink interpretation:
% A simple block diagram with a Constant block for x, a Gain block for A,
% and a Sum block for subtracting b represents the error A*x - b.
      
