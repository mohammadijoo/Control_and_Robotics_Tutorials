function path = rrt_star_2d(xlim, ylim, start, goal, obstacles)
% xlim, ylim: [min max]
% start, goal: [x y]
% obstacles: Mx4, rows [xmin ymin xmax ymax]

stepSize    = 0.1;
goalRadius  = 0.2;
maxIter     = 3000;
gamma_rrt   = 1.0;

nodes(1).x     = start(1);
nodes(1).y     = start(2);
nodes(1).parent = -1;
nodes(1).cost   = 0.0;

for k = 1:maxIter
    x_rand = [ ...
        rand()*(xlim(2)-xlim(1)) + xlim(1), ...
        rand()*(ylim(2)-ylim(1)) + ylim(1) ...
    ];

    idx_near = nearest_node(nodes, x_rand);
    x_near = [nodes(idx_near).x, nodes(idx_near).y];
    x_new  = steer(x_near, x_rand, stepSize);

    if in_obstacle(x_new, obstacles)
        continue;
    end
    if ~line_collision_free(x_near, x_new, obstacles)
        continue;
    end

    nbr_idx = near_nodes(nodes, x_new, gamma_rrt, stepSize);
    if isempty(nbr_idx)
        nbr_idx = idx_near;
    end

    best_parent = idx_near;
    best_cost = nodes(idx_near).cost + norm(x_new - x_near);

    for i = nbr_idx
        xi = [nodes(i).x, nodes(i).y];
        if line_collision_free(xi, x_new, obstacles)
            cand = nodes(i).cost + norm(x_new - xi);
            if cand < best_cost
                best_cost = cand;
                best_parent = i;
            end
        end
    end

    new_idx = numel(nodes) + 1;
    nodes(new_idx).x = x_new(1);
    nodes(new_idx).y = x_new(2);
    nodes(new_idx).parent = best_parent;
    nodes(new_idx).cost = best_cost;

    % rewire
    for i = nbr_idx
        if i == best_parent
            continue;
        end
        xi = [nodes(i).x, nodes(i).y];
        if line_collision_free(x_new, xi, obstacles)
            cand = best_cost + norm(x_new - xi);
            if cand < nodes(i).cost
                nodes(i).parent = new_idx;
                nodes(i).cost = cand;
            end
        end
    end

    if norm(x_new - goal) <= goalRadius
        goal_idx = new_idx; %#ok<NASGU>
        break;
    end
end

% extract path if goal reached
if exist('goal_idx', 'var')
    path = extract_path(nodes, goal, goal_idx);
else
    path = [];
end
end

function idx = nearest_node(nodes, x)
dmin = inf;
idx = 1;
for i = 1:numel(nodes)
    p = [nodes(i).x, nodes(i).y];
    d = norm(p - x);
    if d < dmin
        dmin = d;
        idx = i;
    end
end
end

function x_new = steer(x_near, x_rand, stepSize)
v = x_rand - x_near;
nrm = norm(v);
if nrm <= stepSize
    x_new = x_rand;
else
    x_new = x_near + stepSize * v / nrm;
end
end

function inside = in_obstacle(x, obstacles)
px = x(1); py = x(2);
inside = false;
for i = 1:size(obstacles,1)
    xmin = obstacles(i,1);
    ymin = obstacles(i,2);
    xmax = obstacles(i,3);
    ymax = obstacles(i,4);
    if px >= xmin && px <= xmax && ...
       py >= ymin && py <= ymax
        inside = true;
        return;
    end
end
end

function free = line_collision_free(p, q, obstacles)
steps = 10;
free = true;
for i = 0:steps
    alpha = i / steps;
    x = (1-alpha)*p + alpha*q;
    if in_obstacle(x, obstacles)
        free = false;
        return;
    end
end
end

function idxs = near_nodes(nodes, x_new, gamma_rrt, stepSize)
n = numel(nodes);
if n == 1
    idxs = 1;
    return;
end
d = 2;
r_n = gamma_rrt * (log(n)/n)^(1/d);
r_n = min(r_n, 10*stepSize);
idxs = [];
for i = 1:n
    p = [nodes(i).x, nodes(i).y];
    if norm(p - x_new) <= r_n
        idxs(end+1) = i; %#ok<AGROW>
    end
end
end

function path = extract_path(nodes, goal, goal_idx)
pts = goal;
idx = goal_idx;
while idx ~= -1
    n = nodes(idx);
    pts = [ [n.x; n.y], pts ];
    idx = n.parent;
end
path = pts;
end
      
