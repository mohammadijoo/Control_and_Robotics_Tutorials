
Ts = 0.02;
I  = 1.0;

A = [1 Ts;
     0 1];
B = [0.5*Ts^2/I;
     Ts/I];

Q = diag([10 1]);
R = 0.1;
P = Q;   % could be DARE solution

N = 20;
n = size(A,1);
m = size(B,2);

% Build prediction matrices
A_bar = zeros(N*n, n);
B_bar = zeros(N*n, N*m);

for i = 1:N
    A_power = A^i;
    A_bar((i-1)*n+1:i*n, :) = A_power;
    for j = 1:i
        A_ij = A^(i-j);
        B_bar((i-1)*n+1:i*n, (j-1)*m+1:j*m) = A_ij * B;
    end
end

Q_bar = blkdiag(kron(eye(N-1), Q), P);
R_bar = kron(eye(N), R);

H = B_bar' * Q_bar * B_bar + R_bar;
F = B_bar' * Q_bar * A_bar;

H_inv = inv(H);

mpc_control = @(x) - (H_inv * (F * x))(1);

% Closed-loop simulation
T_final = 1.0;
N_steps = round(T_final / Ts);

x = [0.5; 0.0];
for k = 1:N_steps
    u = mpc_control(x);
    x = A * x + B * u;
    fprintf('k=%d, q=%f, qdot=%f, u=%f\n', k-1, x(1), x(2), u);
end

% In Simulink, you can alternatively use:
%  - "State-Space" block for A,B,C,D
%  - "MPC Controller" block (Model Predictive Control Toolbox)
%  - Connect measured states (or estimates), reference, and constraints.
