function [path, cost] = astar_grid(occGrid, start, goal)
% occGrid: logical matrix, true for obstacle
% start, goal: [row, col]

[nr, nc] = size(occGrid);
INF = inf;

% 4-connected neighborhood
dirs = [ -1 0; 1 0; 0 -1; 0 1 ];

g = INF * ones(nr, nc);
parent = zeros(nr, nc, 2);

    function h = heuristic(cell)
        % Manhattan distance
        h = abs(cell(1) - goal(1)) + abs(cell(2) - goal(2));
    end

open = java.util.PriorityQueue();
startH = heuristic(start);
g(start(1), start(2)) = 0;
open.add({startH, start, 0}); % {f, [r c], g}

closed = false(nr, nc);

while ~open.isEmpty()
    entry = open.poll();
    cell = entry{2};
    r = cell(1); c = cell(2);
    g_val = entry{3};

    if closed(r, c)
        continue;
    end
    if r == goal(1) && c == goal(2)
        % reconstruct path
        path = [r, c];
        while parent(r, c, 1) ~= 0
            pr = parent(r, c, 1);
            pc = parent(r, c, 2);
            path = [pr, pc; path]; %#ok<AGROW>
            r = pr; c = pc;
        end
        cost = g_val;
        return;
    end

    closed(r, c) = true;
    for k = 1:size(dirs, 1)
        rr = r + dirs(k,1);
        cc = c + dirs(k,2);
        if rr < 1 || rr > nr || cc < 1 || cc > nc
            continue;
        end
        if occGrid(rr, cc)
            continue;
        end
        gTent = g_val + 1;
        if gTent < g(rr, cc)
            g(rr, cc) = gTent;
            parent(rr, cc, :) = [r, c];
            f = gTent + heuristic([rr, cc]);
            open.add({f, [rr, cc], gTent});
        end
    end
end

error('No path found');
end
      
