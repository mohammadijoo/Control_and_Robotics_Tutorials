
% Single-joint parameters
I = 0.5;
b = 0.1;

A = [0 1;
     0 -b/I];
B = [0;
     1/I];

% Discretization
dt = 0.002;
T  = 2.0;
N  = round(T/dt);

Ad = eye(2) + A*dt;
Bd = B*dt;

% Weights
Q = diag([100 10]);
R = 1;

% Backward Riccati recursion
P = zeros(2,2,N+1);
K = zeros(1,2,N);

P(:,:,N+1) = Q;   % terminal cost

for k = N:-1:1
    Pnext = P(:,:,k+1);
    S = R + Bd'*Pnext*Bd;
    K(:,:,k) = (S \ (Bd'*Pnext*Ad));
    P(:,:,k) = Q + Ad'*Pnext*Ad - Ad'*Pnext*Bd*K(:,:,k);
end

% Closed-loop simulation
x = [0.3; 0.0];   % initial state
x_log = zeros(2,N);
u_log = zeros(1,N);

for k = 1:N
    u = -squeeze(K(:,:,k)) * x;
    x = x + dt*(A*x + B*u);
    x_log(:,k) = x;
    u_log(:,k) = u;
end
