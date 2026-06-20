% Define a family of second-order joint models
N = 4;
A = cell(1,N);
B = cell(1,N);
Q = eye(2);
R = 0.1;

for i = 1:N
    wn = 2 + 0.5*(i-1);   % natural frequency
    zeta = 0.4 + 0.1*(i-1);
    A{i} = [0 1; -wn^2 -2*zeta*wn];
    B{i} = [0; 1];
end

% Specialist LQR controllers
K_spec = cell(1,N);
J_spec = zeros(1,N);

for i = 1:N
    [K_spec{i},~,~] = lqr(A{i}, B{i}, Q, R);
    % Evaluate infinite-horizon cost for unit covariance initial state
    P = care(A{i} - B{i}*K_spec{i}, B{i}, Q + K_spec{i}'*R*K_spec{i}, R);
    J_spec(i) = trace(P);  % expected quadratic cost proxy
end

% Generalist LQR: solve a single Riccati equation for averaged system
A_bar = zeros(2);
B_bar = zeros(2,1);
for i = 1:N
    A_bar = A_bar + A{i} / N;
    B_bar = B_bar + B{i} / N;
end

[K_gen,~,~] = lqr(A_bar, B_bar, Q, R);
J_gen = zeros(1,N);

for i = 1:N
    P = care(A{i} - B{i}*K_gen, B{i}, Q + K_gen'*R*K_gen, R);
    J_gen(i) = trace(P);
end

fprintf("Avg specialist cost: %.3f\n", mean(J_spec));
fprintf("Avg generalist cost: %.3f\n", mean(J_gen));

% In Simulink, you can:
% 1. Use a single "LQR Controller" block with tunable gain K_gen for a generalist.
% 2. Use a variant subsystem with task-specific gains K_spec{i} for specialists.
      
