function hybrid_tamp_grid
    W = 7; H = 7;
    FREE = 0; OBST = 1;

    grid = FREE * ones(H, W);
    grid(4,4) = OBST; % MATLAB indices are 1-based

    startRobot = [1, 1];   % (x,y)
    startObj   = [3, 3];
    goalObj    = [7, 7];

    MODE_TRANSIT  = 0;
    MODE_TRANSFER = 1;

    start.mode = MODE_TRANSIT;
    start.rx = startRobot(1); start.ry = startRobot(2);
    start.ox = startObj(1);   start.oy = startObj(2);

    % BFS frontier
    frontier = {start};
    parent = containers.Map();
    parent_key = state_key(start);
    parent(parent_key) = struct('parent', '', 'action', 'start');

    goalFound = false;
    goalState = start;

    while ~isempty(frontier)
        s = frontier{1};
        frontier(1) = [];

        if is_goal(s, goalObj)
            goalFound = true;
            goalState = s;
            break;
        end

        succ = neighbors(s, grid, goalObj, MODE_TRANSIT, MODE_TRANSFER, FREE);
        for i = 1:numel(succ)
            ns = succ(i).state;
            act = succ(i).action;
            k = state_key(ns);
            if ~isKey(parent, k)
                parent(k) = struct('parent', state_key(s), 'action', act);
                frontier{end+1} = ns;
            end
        end
    end

    if ~goalFound
        fprintf('No hybrid plan found.\n');
        return;
    end

    % Reconstruct plan
    plan = {};
    curKey = state_key(goalState);
    while ~strcmp(parent(curKey).parent, '')
        par = parent(curKey);
        plan{end+1} = struct('action', par.action, 'key', curKey); %#ok<AGROW>
        curKey = par.parent;
    end
    plan = fliplr(plan);

    fprintf('Plan length: %d\n', numel(plan));
    for i = 1:numel(plan)
        fprintf('%s : %s\n', plan{i}.action, plan{i}.key);
    end
end

function flag = is_goal(s, goalObj)
    flag = (s.mode == 0) && (s.ox == goalObj(1)) && (s.oy == goalObj(2));
end

function k = state_key(s)
    k = sprintf('m%d_rx%d_ry%d_ox%d_oy%d', s.mode, s.rx, s.ry, s.ox, s.oy);
end

function S = neighbors(s, grid, goalObj, MODE_TRANSIT, MODE_TRANSFER, FREE)
    dirs = [1 0; -1 0; 0 1; 0 -1];
    [H, W] = size(grid);
    S = struct('action', {}, 'state', {});

    for i = 1:size(dirs,1)
        nx = s.rx + dirs(i,1);
        ny = s.ry + dirs(i,2);
        if nx < 1 || nx > W || ny < 1 || ny > H
            continue;
        end
        if grid(ny, nx) ~= FREE
            continue;
        end

        ns = s;
        ns.rx = nx; ns.ry = ny;
        if s.mode == MODE_TRANSIT
            % object stays
        else
            ns.ox = nx; ns.oy = ny;
        end
        S(end+1).action = 'move'; %#ok<AGROW>
        S(end).state = ns;
    end

    if s.mode == MODE_TRANSIT && s.rx == s.ox && s.ry == s.oy
        ns = s;
        ns.mode = MODE_TRANSFER;
        S(end+1).action = 'pick'; %#ok<AGROW>
        S(end).state = ns;
    end

    if s.mode == MODE_TRANSFER && s.rx == goalObj(1) && s.ry == goalObj(2)
        ns = s;
        ns.mode = MODE_TRANSIT;
        S(end+1).action = 'place'; %#ok<AGROW>
        S(end).state = ns;
    end
end
      
