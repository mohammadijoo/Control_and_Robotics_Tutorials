function hri_metrics_demo()
    % Compute Fitts throughput, Hick-Hyman RT, and safe approach speed.
    % In Simulink, the same equations can be implemented using basic math blocks.

    % Example pointing data
    D = [0.20 0.30 0.40];   % meters
    W = [0.04 0.05 0.06];   % meters
    MT = [0.45 0.50 0.55];  % seconds

    ID = log2(1 + D ./ W);
    TP = sum(ID) / sum(MT);

    % Hick-Hyman reaction time with non-uniform probabilities
    p = [0.4 0.3 0.2 0.1];
    p = p / sum(p);
    H = -sum(p .* log2(p));
    a = 0.25; b = 0.15;
    t_r = a + b * H;

    % Safe approach speed
    a_max = 3.0;   % m/s^2
    d_safe = 1.5;  % m
    [v_max] = safe_velocity(t_r, a_max, d_safe);

    fprintf('Throughput TP = %.3f bits/s\n', TP);
    fprintf('Reaction time t_r = %.3f s\n', t_r);
    fprintf('Max safe speed v_max = %.3f m/s\n', v_max);

    % Control-style human-in-the-loop model (for Simulink)
    K_r = 1; T_r = 0.5;
    K_h = 1; T_d = 0.3;
    s = tf('s');
    G = K_r / (T_r * s + 1);
    Hs = K_h * (1 - s*T_d/2) / (1 + s*T_d/2); % Pade approximation of delay
    T_closed = feedback(G*Hs, 1);
    disp('Closed-loop poles with human in the loop:');
    disp(pole(T_closed));
end

function v_max = safe_velocity(t_r, a_max, d_safe)
    % Solve v^2/(2 a_max) + t_r v - d_safe = 0 for smallest v >= 0.
    A = 1.0 / (2.0 * a_max);
    B = t_r;
    C = -d_safe;
    disc = B^2 - 4*A*C;
    if disc < 0
        v_max = 0.0;
        return;
    end
    v1 = (-B + sqrt(disc)) / (2*A);
    v2 = (-B - sqrt(disc)) / (2*A);
    candidates = [v1 v2];
    candidates = candidates(candidates >= 0);
    if isempty(candidates)
        v_max = 0.0;
    else
        v_max = min(candidates);
    end
end
      
