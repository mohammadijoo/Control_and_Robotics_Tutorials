% Chapter6_Lesson2.m
% Bayes Filters for Mobile Robots (discrete-state version)
%
% Implements:
%   bel_bar = T * bel_prev
%   bel     = normalize(L .* bel_bar)
%
% Also includes a 1D ring corridor demo.

function Chapter6_Lesson2()
    rng(4);

    N = 20;
    bel = ones(N,1) / N;

    landmarks = [3; 14]; % 0-indexed landmark positions (we will mod N)
    controls  = [1 1 1 2 1 1 2 1];

    x = randi([0, N-1]);
    truth = zeros(numel(controls)+1,1);
    truth(1) = x;
    zs = zeros(numel(controls),1);

    % Simulate
    for t = 1:numel(controls)
        u = controls(t);
        r = rand();
        if r < 0.8
            x = mod(x + u, N);
        elseif r < 0.9
            x = mod(x + u - 1, N);
        else
            x = mod(x + u + 1, N);
        end
        truth(t+1) = x;

        at_landmark = any(x == mod(landmarks, N));
        if at_landmark
            z = double(rand() < 0.9);
        else
            z = double(rand() < 0.1);
        end
        zs(t) = z;
    end

    % Filter
    for t = 1:numel(controls)
        u = controls(t);
        z = zs(t);

        T = make_shift_transition(N, u, 0.8, 0.1, 0.1);
        L = landmark_likelihood(N, z, landmarks, 0.9, 0.1);

        bel_bar = normalize(T * bel);
        bel = normalize(L .* bel_bar);
    end

    % Report top 5
    [~, idx] = sort(bel, 'descend');
    fprintf('Final belief (top 5 states):\n');
    for k = 1:5
        i = idx(k) - 1; % back to 0-indexed for display
        fprintf('  state %2d: %.4f\n', i, bel(idx(k)));
    end
    fprintf('Truth trajectory: ');
    fprintf('%d', truth(1));
    for i = 2:numel(truth)
        fprintf(', %d', truth(i));
    end
    fprintf('\n');
end

function p = normalize(p)
    s = sum(p);
    if s < 1e-12
        p = ones(size(p)) / numel(p);
    else
        p = p / s;
    end
end

function T = make_shift_transition(N, delta, p_exact, p_under, p_over)
    if abs((p_exact + p_under + p_over) - 1.0) > 1e-9
        error('Probabilities must sum to 1');
    end
    T = zeros(N, N);
    for j = 0:(N-1)
        i_exact = mod(j + delta, N);
        i_under = mod(j + delta - 1, N);
        i_over  = mod(j + delta + 1, N);
        T(i_exact+1, j+1) = T(i_exact+1, j+1) + p_exact;
        T(i_under+1, j+1) = T(i_under+1, j+1) + p_under;
        T(i_over+1,  j+1) = T(i_over+1,  j+1) + p_over;
    end
end

function L = landmark_likelihood(N, z, landmarks, p_hit, p_false)
    if z == 1
        base = p_false;
    else
        base = 1.0 - p_false;
    end
    L = base * ones(N, 1);

    for k = 1:numel(landmarks)
        x = mod(landmarks(k), N);
        if z == 1
            L(x+1) = p_hit;
        else
            L(x+1) = 1.0 - p_hit;
        end
    end
end
