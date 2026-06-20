function goalNode = bitstar2d(start, goal, bounds, nBatches, batchSize, r0)
% start, goal: 1x2 row vectors
% bounds: [xmin xmax ymin ymax]
% Return: struct with fields x, parent, g

if nargin < 6, r0 = 1.0; end
INF = 1e9;

root.x = start;
root.parent = 0;
root.g = 0.0;
tree = root;
bestCost = INF;
goalNode = [];

for k = 1:nBatches
    % Sample batch (no obstacles in this simple example)
    samples = zeros(batchSize, 2);
    for i = 1:batchSize
        samples(i,:) = sampleInformed(start, goal, bestCost, bounds);
    end

    nTotal = numel(tree) + batchSize;
    r = r0 * (log(max(nTotal, 2)) / nTotal)^(1/2);

    % Build edge queue: [f_lb, parentIndex, sampleIndex]
    edgeQueue = [];
    for i = 1:batchSize
        x = samples(i,:);
        for j = 1:numel(tree)
            xp = tree(j).x;
            d = norm(xp - x);
            if d <= r
                gParent = tree(j).g;
                h = norm(x - goal);
                f_lb = gParent + d + h;
                if f_lb < bestCost
                    edgeQueue = [edgeQueue; f_lb, j, i]; %#ok<AGROW>
                end
            end
        end
    end

    % Process edges by increasing f_lb
    if isempty(edgeQueue), continue; end
    edgeQueue = sortrows(edgeQueue, 1);

    for e = 1:size(edgeQueue, 1)
        f_lb = edgeQueue(e,1);
        if f_lb >= bestCost
            break;
        end
        parentIdx = edgeQueue(e,2);
        sampIdx   = edgeQueue(e,3);
        x = samples(sampIdx,:);

        % Check if x already in tree
        already = false;
        for j = 1:numel(tree)
            if norm(tree(j).x - x) < 1e-6
                already = true;
                break;
            end
        end
        if already, continue; end

        xp = tree(parentIdx).x;
        % TODO: collision check between xp and x
        gNew = tree(parentIdx).g + norm(xp - x);

        newNode.x = x;
        newNode.parent = parentIdx;
        newNode.g = gNew;
        tree(end + 1) = newNode; %#ok<AGROW>

        if norm(x - goal) <= r
            % Direct connection to goal
            gGoal = gNew + norm(x - goal);
            if gGoal < bestCost
                bestCost = gGoal;
                goalNode.x = goal;
                goalNode.parent = numel(tree);
                goalNode.g = gGoal;
            end
        end
    end
end
end

function x = sampleInformed(start, goal, cBest, bounds)
if ~isfinite(cBest)
    % uniform sampling
    xmin = bounds(1); xmax = bounds(2);
    ymin = bounds(3); ymax = bounds(4);
    x = [xmin + (xmax - xmin) * rand, ...
         ymin + (ymax - ymin) * rand];
    return;
end
% Elliptical informed sampling (similar to Python version)
xs = start(1); ys = start(2);
xg = goal(1);  yg = goal(2);
df = norm(start - goal);
if cBest <= df
    x = start;
    return;
end
cx = 0.5 * (xs + xg);
cy = 0.5 * (ys + yg);
theta = atan2(yg - ys, xg - xs);
a = 0.5 * cBest;
c = 0.5 * df;
b = sqrt(max(a^2 - c^2, 0));

while true
    u = 2 * rand - 1;
    v = 2 * rand - 1;
    if u^2 + v^2 <= 1
        break;
    end
end
lx = a * u;
ly = b * v;
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
w = R * [lx; ly];
x = [cx; cy] + w;
x = x(:).';
end
      
