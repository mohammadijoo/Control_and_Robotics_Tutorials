% State: [q1; q2; dq1; dq2], Action: [tau1; tau2]
STATE_DIM = 4;
ACTION_DIM = 2;

function tau = expert_policy(s)
    q_des = [0.5; -0.5];
    q = s(1:2);
    dq = s(3:4);
    Kp = 5.0; Kd = 1.0;
    tau = Kp * (q_des - q) - Kd * dq;
end

function s_next = simulate_step(s, tau, dt)
    q = s(1:2);
    dq = s(3:4);
    ddq = tau; % placeholder, use rigidBodyTree dynamics in practice
    dq_next = dq + ddq * dt;
    q_next = q + dq_next * dt;
    s_next = [q_next; dq_next];
end

function [S, A] = collect_expert_rollout(T)
    s = zeros(STATE_DIM, 1);
    S = zeros(STATE_DIM, T);
    A = zeros(ACTION_DIM, T);
    dt = 0.02;
    for t = 1:T
        tau = expert_policy(s);
        S(:, t) = s;
        A(:, t) = tau;
        s = simulate_step(s, tau, dt);
    end
end

% Behavior cloning via linear least squares: A ≈ W*S + b
T = 200;
[S, A] = collect_expert_rollout(T);
X = [S' ones(T,1)];          % [s', 1]
Y = A';                      % actions
theta = (X' * X) \ (X' * Y); % (STATE_DIM+1) x ACTION_DIM
W = theta(1:STATE_DIM, :)';  % 2 x 4
b = theta(STATE_DIM+1, :)';  % 2 x 1

% DAgger iterations
numIters = 5;
for k = 1:numIters
    % Rollout current policy
    dt = 0.02;
    s = zeros(STATE_DIM, 1);
    S_new = zeros(STATE_DIM, T);
    A_new = zeros(ACTION_DIM, T);
    for t = 1:T
        tau_pi = W * s + b;       % current policy
        tau_exp = expert_policy(s); % query expert at visited state
        S_new(:, t) = s;
        A_new(:, t) = tau_exp;
        s = simulate_step(s, tau_pi, dt);
    end
    % Aggregate and retrain
    S = [S, S_new];
    A = [A, A_new];
    X = [S' ones(size(S,2),1)];
    Y = A';
    theta = (X' * X) \ (X' * Y);
    W = theta(1:STATE_DIM, :)';
    b = theta(STATE_DIM+1, :)';
    fprintf('DAgger iter %d, dataset size = %d\n', k, size(S,2));
end

% To deploy in Simulink:
% 1. Create a MATLAB Function block implementing tau = W*s + b.
% 2. Feed joint states into this block and connect outputs to your torque inputs.
      
