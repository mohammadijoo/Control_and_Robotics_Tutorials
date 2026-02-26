
% Linearized manipulator dynamics: x_{k+1} = A_d x_k + B_d u_k
A_d = ...;  % from c2d(linearized continuous-time model, Ts)
B_d = ...;

Q = diag([100 100 1 1]);
R = 0.01 * eye(2);

% Infinite-horizon LQR
[K_inf, P_inf, ~] = dlqr(A_d, B_d, Q, R);

% Finite-horizon LQR with horizon N
N = 30;        % choose small N to reduce computation
P = cell(N+1, 1);
K = cell(N, 1);
P{N+1} = P_inf;    % terminal cost approximation

for k = N:-1:1
    Ak = A_d;
    Bk = B_d;
    Pkp1 = P{k+1};

    S = R + Bk' * Pkp1 * Bk;
    Kk = -S \ (Bk' * Pkp1 * Ak);

    Pk = Q + Ak' * Pkp1 * Ak ...
           + Ak' * Pkp1 * Bk * Kk ...
           + Kk' * Bk' * Pkp1 * Ak ...
           + Kk' * R * Kk;

    K{k} = Kk;
    P{k} = Pk;
end

% Real-time implementation:
% At each sample, use K{1} as feedback gain (or time-varying K{k} along a trajectory).
% Simulink implementation can embed these gains in a discrete-time state-feedback block.
