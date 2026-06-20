gamma = 0.99;
alpha = 0.1;      % Q-learning step size
lambda_lr = 1e-2;
lambda_c = 0.0;
cost_limit = 0.5;

nStates = 20;
nActions = 3;

Q_r = zeros(nStates, nActions);  % reward Q
Q_c = zeros(nStates, nActions);  % constraint Q

env = MySafeRobotEnv();  % user-defined MATLAB class

for episode = 1:1000
    s = reset(env);  % state index
    done = false;
    ep_cost = 0.0;

    while ~done
        % epsilon-greedy action selection
        if rand() < 0.1
            a = randi(nActions);
        else
            [~, a] = max(Q_r(s,:) - lambda_c * Q_c(s,:));
        end

        [s_next, r, c, done] = step(env, a);
        ep_cost = ep_cost + c;

        % standard Q-learning updates
        best_next = max(Q_r(s_next,:) - lambda_c * Q_c(s_next,:));

        td_target_r = r + gamma * best_next;
        td_error_r = td_target_r - Q_r(s,a);
        Q_r(s,a) = Q_r(s,a) + alpha * td_error_r;

        td_target_c = c + gamma * max(Q_c(s_next,:));
        td_error_c = td_target_c - Q_c(s,a);
        Q_c(s,a) = Q_c(s,a) + alpha * td_error_c;

        s = s_next;
    end

    % Dual variable update at end of episode
    lambda_c = max(0.0, lambda_c + lambda_lr * (ep_cost - cost_limit));

    fprintf("Episode %d, cost %.3f, lambda %.3f\n", ...
            episode, ep_cost, lambda_c);
end
      
