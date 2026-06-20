function path = astar_grid(start, goal, isFree)
% A* on a 2D grid with 4-connected moves.
% start, goal: [x; y]
% isFree(x, y): function handle returning true if cell is free

    start = start(:).';
    goal  = goal(:).';

    g = containers.Map();
    parent = containers.Map();
    keyStart = key(start);
    g(keyStart) = 0.0;
    parent(keyStart) = '';

    % open list: [f, x, y]
    open = [heuristic(start, goal), start];
    closed = containers.Map();

    while ~isempty(open)
        % pop node with minimal f
        [~, idx] = min(open(:,1));
        node = open(idx,:);
        open(idx,:) = [];

        s = node(2:3);
        k = key(s);
        if isKey(closed, k)
            continue;
        end
        closed(k) = true;

        if isequal(s, goal)
            path = reconstruct_path(parent, s);
            return;
        end

        g_s = g(k);
        nbrs = successorsAStar(s);
        for i = 1:size(nbrs,1)
            sn = nbrs(i,:);
            if ~isFree(sn(1), sn(2))
                continue;
            end
            kn = key(sn);
            c = 1.0; % uniform edge cost
            new_g = g_s + c;
            if ~isKey(g, kn) || new_g < g(kn)
                g(kn) = new_g;
                parent(kn) = k;
                f = new_g + heuristic(sn, goal);
                open = [open; f, sn];
            end
        end
    end

    path = []; % no solution
end

function h = heuristic(s, goal)
    dx = abs(s(1) - goal(1));
    dy = abs(s(2) - goal(2));
    h = dx + dy; % Manhattan distance
end

function nbrs = successorsAStar(s)
    steps = [1 0; -1 0; 0 1; 0 -1];
    nbrs = bsxfun(@plus, steps, s);
end

function k = key(s)
    k = sprintf('%d_%d', s(1), s(2));
end

function path = reconstruct_path(parent, s)
    seq = s;
    k = key(s);
    while parent.isKey(k) && ~isempty(parent(k))
        k = parent(k);
        tokens = sscanf(k, '%d_%d');
        seq = [tokens.'; seq];
    end
    path = seq;
end
      
