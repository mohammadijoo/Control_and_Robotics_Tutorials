function path = rrt2d(qStart, qGoal, bounds, obstacles, eta, goalRadius, maxIter)
% qStart, qGoal: 1x2 row vectors
% bounds: [xmin xmax ymin ymax]
% obstacles: Nx4 rectangles [xmin xmax ymin ymax]

nodes = qStart;
parent = -1;  % parent(1) = -1 (root)

for k = 1:maxIter
    % Goal bias
    if rand() < 0.1
        qRand = qGoal;
    else
        qRand = [ ...
            bounds(1) + rand() * (bounds(2) - bounds(1)), ...
            bounds(3) + rand() * (bounds(4) - bounds(3)) ];
    end

    idxNear = nearestNode(nodes, qRand);
    qNear = nodes(idxNear, :);
    qNew = steer(qNear, qRand, eta);

    if collisionFree(qNear, qNew, obstacles)
        nodes(end+1, :) = qNew; %#ok<AGROW>
        parent(end+1, 1) = idxNear; %#ok<AGROW>
        if norm(qNew - qGoal) <= goalRadius
            % Reconstruct path
            idx = size(nodes,1);
            path = zeros(0, 2);
            while idx ~= -1
                path = [nodes(idx,:); path]; %#ok<AGROW>
                idx = parent(idx);
            end
            return;
        end
    end
end

path = []; % failure
end

function idx = nearestNode(nodes, q)
    diffs = nodes - q;
    d2 = sum(diffs.^2, 2);
    [~, idx] = min(d2);
end

function qNew = steer(qNear, qRand, eta)
    d = qRand - qNear;
    dist = norm(d);
    if dist <= eta
        qNew = qRand;
    else
        qNew = qNear + (eta / dist) * d;
    end
end

function free = collisionFree(q1, q2, obstacles)
    nSteps = 20;
    free = true;
    for i = 0:nSteps
        alpha = i / nSteps;
        q = (1 - alpha) * q1 + alpha * q2;
        x = q(1); y = q(2);
        for j = 1:size(obstacles,1)
            xmin = obstacles(j,1); xmax = obstacles(j,2);
            ymin = obstacles(j,3); ymax = obstacles(j,4);
            if x >= xmin && x <= xmax && y >= ymin && y <= ymax
                free = false;
                return;
            end
        end
    end
end
      
