
nx = 4; nu = 2; ny = 2; N = 10;

A = eye(nx) + 0.01 * randn(nx);
B = 0.01 * randn(nx, nu);
C = [eye(ny), zeros(ny, nx - ny)];

Qy = eye(ny);
R  = 0.01 * eye(nu);

Q_blk = kron(eye(N), Qy);
R_blk = kron(eye(N), R);

% Build prediction matrices Sx, Su
Sx = zeros(N*nx, nx);
Su = zeros(N*nx, N*nu);
A_power = eye(nx);
for i = 1:N
    A_power = A_power * A;
    Sx((i-1)*nx+1:i*nx, :) = A_power;
    for j = 1:i
        A_pow = A^(i-j);
        Su((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = A_pow * B;
    end
end

C_blk = kron(eye(N), C);
T = C_blk * Su;

xk = zeros(nx, 1);
y_ref = zeros(N*ny, 1);

b = C_blk * (Sx * xk) - y_ref;

H = 2 * (T' * Q_blk * T + R_blk);
f = 2 * (T' * Q_blk * b);

% Unconstrained solution: U* = -H\f
U_star = -H \ f;
u0_star = U_star(1:nu);
disp('u0* = ');
disp(u0_star);

% Simulink hint:
%  - Implement the A,B,C model as a State-Space block.
%  - Implement the MPC computation in a MATLAB Function block,
%    which takes x_k, y_ref as inputs and outputs u_k* each sample.
