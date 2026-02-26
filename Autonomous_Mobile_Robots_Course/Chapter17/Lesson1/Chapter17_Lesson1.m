% Autonomous Mobile Robots — Chapter 17, Lesson 1: Frontier-Based Exploration
% File: Chapter17_Lesson1.m
%
% MATLAB reference implementation for frontier detection + clustering + scoring.
% Grid encoding:
%   -1 = unknown, 0 = free, 100 = occupied
%
% Robotics System Toolbox note:
% - For real robots, use occupancyMap/binaryOccupancyMap and planners (plannerAStarGrid,
%   nav2 via ROS2 interface, etc.). Here we keep it matrix-based for clarity.

function Chapter17_Lesson1()
    H = 30; W = 40;
    grid = -1 * ones(H, W);

    % Known free region
    grid(6:15, 6:18) = 0; % MATLAB 1-indexed
    % Obstacles
    grid(6:15, 13) = 100;
    grid(11, 13:30) = 100;
    % Another known pocket
    grid(19:25, 26:35) = 0;

    robot = [9, 9]; % (row, col) in 1-index coordinates

    frontierCells = detect_frontiers(grid);
    clusters = cluster_frontiers(grid, frontierCells);
    dist = bfs_dist_free(grid, robot);

    minClusterSize = 6;
    bestGoal = [];
    bestCost = inf;

    for k = 1:numel(clusters)
        comp = clusters{k};
        if size(comp,1) < minClusterSize
            continue;
        end
        goal = choose_goal_near_centroid(grid, dist, comp, 6);
        if isempty(goal)
            continue;
        end
        d = dist(goal(1), goal(2));
        J = score_cluster(d, size(comp,1), 1.0, 2.5);
        if J < bestCost
            bestCost = J;
            bestGoal = goal;
        end
    end

    disp(['robot=(' num2str(robot(1)) ',' num2str(robot(2)) ')']);
    disp(['frontier_clusters=' num2str(numel(clusters))]);
    if ~isempty(bestGoal)
        disp(['best_goal=(' num2str(bestGoal(1)) ',' num2str(bestGoal(2)) '), cost=' num2str(bestCost)]);
    else
        disp('No reachable frontier goal found.');
    end
end

function frontiers = detect_frontiers(grid)
    [H, W] = size(grid);
    frontiers = zeros(0,2);
    for r = 1:H
        for c = 1:W
            if grid(r,c) ~= 0
                continue;
            end
            nbrs = neighbors4(H, W, [r,c]);
            for i = 1:size(nbrs,1)
                rr = nbrs(i,1); cc = nbrs(i,2);
                if grid(rr,cc) == -1
                    frontiers(end+1,:) = [r,c]; %#ok<AGROW>
                    break;
                end
            end
        end
    end
end

function clusters = cluster_frontiers(grid, frontierCells)
    [H, W] = size(grid);
    if isempty(frontierCells)
        clusters = {};
        return;
    end

    frontierSet = containers.Map();
    for i = 1:size(frontierCells,1)
        key = sprintf('%d,%d', frontierCells(i,1), frontierCells(i,2));
        frontierSet(key) = true;
    end

    visited = containers.Map();
    clusters = {};

    for i = 1:size(frontierCells,1)
        cell = frontierCells(i,:);
        key0 = sprintf('%d,%d', cell(1), cell(2));
        if isKey(visited, key0); continue; end
        if ~isKey(frontierSet, key0); continue; end

        q = cell; % queue as Nx2
        visited(key0) = true;
        comp = zeros(0,2);

        while ~isempty(q)
            u = q(1,:);
            q(1,:) = [];
            comp(end+1,:) = u; %#ok<AGROW>
            nbrs = neighbors8(H, W, u);
            for j = 1:size(nbrs,1)
                v = nbrs(j,:);
                key = sprintf('%d,%d', v(1), v(2));
                if isKey(frontierSet, key) && ~isKey(visited, key)
                    visited(key) = true;
                    q(end+1,:) = v; %#ok<AGROW>
                end
            end
        end

        clusters{end+1} = comp; %#ok<AGROW>
    end

    % sort by size desc
    sizes = cellfun(@(x) size(x,1), clusters);
    [~, idx] = sort(sizes, 'descend');
    clusters = clusters(idx);
end

function dist = bfs_dist_free(grid, start)
    [H, W] = size(grid);
    dist = inf(H, W);
    if grid(start(1), start(2)) ~= 0
        return;
    end

    q = start; % queue Nx2
    dist(start(1), start(2)) = 0;

    while ~isempty(q)
        u = q(1,:);
        q(1,:) = [];
        du = dist(u(1), u(2));
        nbrs = neighbors4(H, W, u);
        for i = 1:size(nbrs,1)
            v = nbrs(i,:);
            if grid(v(1), v(2)) ~= 0
                continue;
            end
            if isinf(dist(v(1), v(2)))
                dist(v(1), v(2)) = du + 1;
                q(end+1,:) = v; %#ok<AGROW>
            end
        end
    end
end

function goal = choose_goal_near_centroid(grid, dist, comp, maxRad)
    [H, W] = size(grid);
    cr = mean(comp(:,1));
    cc = mean(comp(:,2));
    r0 = round(cr); c0 = round(cc);

    goal = [];
    bestD = inf;

    for rad = 0:maxRad
        for r = (r0-rad):(r0+rad)
            for c = (c0-rad):(c0+rad)
                if r < 1 || r > H || c < 1 || c > W
                    continue;
                end
                if grid(r,c) ~= 0
                    continue;
                end
                d = dist(r,c);
                if ~isfinite(d)
                    continue;
                end
                if d < bestD
                    bestD = d;
                    goal = [r,c];
                end
            end
        end
        if ~isempty(goal)
            return;
        end
    end
end

function J = score_cluster(distance, sz, alpha, beta)
    J = alpha * distance - beta * sz;
end

function nbrs = neighbors4(H, W, u)
    r = u(1); c = u(2);
    cand = [r-1 c; r+1 c; r c-1; r c+1];
    mask = cand(:,1) >= 1 & cand(:,1) <= H & cand(:,2) >= 1 & cand(:,2) <= W;
    nbrs = cand(mask,:);
end

function nbrs = neighbors8(H, W, u)
    r = u(1); c = u(2);
    nbrs = zeros(0,2);
    for dr = -1:1
        for dc = -1:1
            if dr == 0 && dc == 0; continue; end
            rr = r + dr; cc = c + dc;
            if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                nbrs(end+1,:) = [rr cc]; %#ok<AGROW>
            end
        end
    end
end
