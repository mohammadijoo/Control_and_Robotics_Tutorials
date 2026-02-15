function path = prm_plan_2d(q_start, q_goal, bounds, obstacles, n_samples, r)
% PRM-based planner in 2D (educational).
% bounds: [xmin xmax; ymin ymax]
% obstacles: N x 4, each row [xmin ymin xmax ymax]
    if nargin < 6
        r = 0.15;
    end
    if nargin < 5
        n_samples = 200;
    end

    % Sample free configurations
    samples = sample_free(n_samples, bounds, obstacles);

    % Append start and goal
    samples = [samples; q_start(:).'; q_goal(:).'];
    idx_start = size(samples, 1) - 1;
    idx_goal  = size(samples, 1);

    % Build roadmap with collision checking (standard PRM)
    G = build_prm(samples, r, obstacles);

    % Dijkstra search
    path_idx = dijkstra_graph(G, idx_start, idx_goal);
    if isempty(path_idx)
        path = [];
    else
        path = samples(path_idx, :);
    end
end

function samples = sample_free(n_samples, bounds, obstacles)
    xs = rand(n_samples * 2, 1) * (bounds(1,2) - bounds(1,1)) + bounds(1,1);
    ys = rand(n_samples * 2, 1) * (bounds(2,2) - bounds(2,1)) + bounds(2,1);
    samples = zeros(n_samples, 2);
    c = 0;
    for i = 1:numel(xs)
        q = [xs(i), ys(i)];
        if ~in_collision(q, obstacles)
            c = c + 1;
            samples(c, :) = q;
            if c == n_samples
                break;
            end
        end
    end
end

function flag = in_collision(q, obstacles)
    x = q(1); y = q(2);
    flag = false;
    for i = 1:size(obstacles, 1)
        box = obstacles(i, :);
        if box(1) <= x && x <= box(3) && ...
           box(2) <= y && y <= box(4)
            flag = true;
            return;
        end
    end
end

function G = build_prm(samples, r, obstacles)
    n = size(samples, 1);
    G = cell(n, 1);
    for i = 1:n
        G{i} = [];
    end
    for i = 1:n
        for j = i+1:n
            d = euclidean(samples(i,:), samples(j,:));
            if d <= r
                if edge_collision_free(samples(i,:), samples(j,:), obstacles)
                    G{i} = [G{i}; j, d];
                    G{j} = [G{j}; i, d];
                end
            end
        end
    end
end

function d = euclidean(a, b)
    diff = a - b;
    d = sqrt(sum(diff.^2));
end

function ok = edge_collision_free(a, b, obstacles)
    nSteps = 20;
    ok = true;
    for i = 0:nSteps
        t = i / nSteps;
        q = (1 - t) * a + t * b;
        if in_collision(q, obstacles)
            ok = false;
            return;
        end
    end
end

function path = dijkstra_graph(G, s, g)
    n = numel(G);
    dist = inf(n, 1);
    prev = -ones(n, 1);
    visited = false(n, 1);
    dist(s) = 0;
    for k = 1:n
        % find unvisited node with smallest dist
        [~, u] = min(dist + visited * max(dist) * 2);
        if visited(u)
            break;
        end
        visited(u) = true;
        if u == g
            break;
        end
        neighbors = G{u};
        for i = 1:size(neighbors, 1)
            v = neighbors(i, 1);
            w = neighbors(i, 2);
            if ~visited(v)
                nd = dist(u) + w;
                if nd < dist(v)
                    dist(v) = nd;
                    prev(v) = u;
                end
            end
        end
    end
    if ~isfinite(dist(g))
        path = [];
        return;
    end
    % reconstruct
    path_idx = g;
    u = g;
    while prev(u) ~= -1
        u = prev(u);
        path_idx = [u; path_idx]; %#ok<AGROW>
    end
    path = path_idx;
end
      
