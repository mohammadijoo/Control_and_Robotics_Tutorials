function pomdp_demo()
    % States: 0 (near door), 1 (far)
    % Belief vector b
    b = [0.5; 0.5];

    % Transition matrices T{a}
    T_stay = [0.9, 0.0;
              0.1, 1.0];
    T_step = [0.1, 0.0;
              0.9, 1.0];

    % Observation vectors Z{o}
    Z_door    = [0.2; 0.9];
    Z_nodoor  = [0.8; 0.1];

    gamma = 0.95;
    horizon = 3;

    % Example: take action "step" and observe "door"
    b = belief_update(b, T_step, Z_door);

    fprintf('Belief after step + door: [%f, %f]\n', b(1), b(2));

    % Evaluate a trivial one-step policy for illustration
    V = value_horizon(b, horizon, T_stay, T_step, Z_door, Z_nodoor, gamma);
    fprintf('Approximate value at belief: %f\n', V);
end

function b_new = belief_update(b, T, Z)
    % Prediction
    b_pred = T' * b;
    % Update
    b_post = Z .* b_pred;
    s = sum(b_post);
    if s == 0
        b_new = [0.5; 0.5];
    else
        b_new = b_post / s;
    end
end

function V = value_horizon(b, depth, T_stay, T_step, Z_door, Z_nodoor, gamma)
    if depth == 0
        V = 0.0;
        return;
    end
    % Rewards
    R_stay = [-0.05; -0.05];
    R_step = [-0.5; +1.0];

    % Actions: 1 = stay, 2 = step
    V_actions = zeros(2, 1);

    % "stay"
    a = 1;
    T = T_stay;
    r = b' * R_stay;
    V_actions(a) = r + gamma * expected_future(b, T, Z_door, Z_nodoor, depth - 1, ...
                                               T_stay, T_step, gamma);

    % "step"
    a = 2;
    T = T_step;
    r = b' * R_step;
    V_actions(a) = r + gamma * expected_future(b, T, Z_door, Z_nodoor, depth - 1, ...
                                               T_stay, T_step, gamma);

    V = max(V_actions);
end

function fut = expected_future(b, T, Z_door, Z_nodoor, depth, T_stay, T_step, gamma)
    % Compute predictive belief
    b_pred = T' * b;

    % Probabilities of each observation
    p_door   = (b_pred' * Z_door);
    p_nodoor = (b_pred' * Z_nodoor);

    fut = 0.0;
    if p_door > 0
        b_post = belief_update(b, T, Z_door);
        fut = fut + p_door * value_horizon(b_post, depth, T_stay, T_step, ...
                                           Z_door, Z_nodoor, gamma);
    end
    if p_nodoor > 0
        b_post = belief_update(b, T, Z_nodoor);
        fut = fut + p_nodoor * value_horizon(b_post, depth, T_stay, T_step, ...
                                             Z_door, Z_nodoor, gamma);
    end
end
      
