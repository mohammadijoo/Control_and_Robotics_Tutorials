UNKNOWN = -1;
FREE = 0;
OBSTACLE = 1;

function idx = selectClosestFrontier(occGrid, robotIdx)
    ri = robotIdx(1);
    rj = robotIdx(2);
    [rows, cols] = size(occGrid);
    bestDist = inf;
    idx = [];
    for i = 1:rows
        for j = 1:cols
            if isFrontier(occGrid, i, j, FREE, UNKNOWN)
                dx = i - ri;
                dy = j - rj;
                d = sqrt(dx*dx + dy*dy);
                if d < bestDist
                    bestDist = d;
                    idx = [i, j];
                end
            end
        end
    end
end

function flag = isFrontier(occGrid, i, j, FREE, UNKNOWN)
    if occGrid(i, j) ~= FREE
        flag = false; return;
    end
    [rows, cols] = size(occGrid);
    neigh = [-1 0; 1 0; 0 -1; 0 1];
    flag = false;
    for k = 1:4
        ni = i + neigh(k,1);
        nj = j + neigh(k,2);
        if ni >= 1 && ni <= rows && nj >= 1 && nj <= cols
            if occGrid(ni, nj) == UNKNOWN
                flag = true; return;
            end
        end
    end
end
      
