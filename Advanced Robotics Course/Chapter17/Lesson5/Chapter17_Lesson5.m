function [Ad,Bd,Cd,Dd] = buildCableBendingModel(M, dt, mass, k_bend)
%BUILD CABLE BENDING MODEL
% M: number of segments (M+1 nodes)
% dt: time step
% mass: point mass at each node
% k_bend: bending stiffness

N = M + 1;
nq = N; % 1D positions for simplicity
nv = N; % velocities
n  = nq + nv;

% second-difference matrix D (N-2 x N)
D = zeros(N-2, N);
for k = 2:N-1
    D(k-1, k-1) = 1.0;
    D(k-1, k)   = -2.0;
    D(k-1, k+1) = 1.0;
end

K = k_bend * (D' * D);    % stiffness matrix
Mmat = mass * eye(N);     % mass matrix

% state x = [q; v]
A_cont = zeros(n, n);
A_cont(1:nq, nq+1:end) = eye(N);           % qdot = v
A_cont(nq+1:end, 1:nq) = -Mmat \ K;       % vdot = -M^{-1} K q

B_cont = zeros(n, 1);    % no external input here

% discretize with Euler
Ad = eye(n) + dt * A_cont;
Bd = dt * B_cont;
Cd = eye(n);
Dd = zeros(n, 1);
      
