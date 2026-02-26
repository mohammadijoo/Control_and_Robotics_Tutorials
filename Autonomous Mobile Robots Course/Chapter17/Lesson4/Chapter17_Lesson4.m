% Chapter17_Lesson4.m
% Exploration Under Limited Battery/Time (Budget-Aware Frontier Exploration)
% Self-contained MATLAB demo (no toolboxes required).
%
% Notes for Simulink:
% - You can implement the "budget update" and "goal selection" blocks in Simulink
%   using MATLAB Function blocks, then connect a discrete-time integrator for
%   time and energy and a state machine (Stateflow) for Explore/Return states.

clear; clc; rng(7);

W = 35; H = 25;
belief = 0.5 * ones(H, W);        % p_occ belief
truth  = 0.05 * ones(H, W);       % synthetic truth (mostly free)

% Borders as obstacles
truth(1,:) = 0.95; truth(end,:) = 0.95;
truth(:,1) = 0.95; truth(:,end) = 0.95;

% Random rectangles
rng(3);
for k=1:8
    x0 = randi([4, W-10]); y0 = randi([4, H-8]);
    ww = randi([3, 7]);    hh = randi([2, 5]);
    x1 = min(W-1, x0+ww);  y1 = min(H-1, y0+hh);
    truth(y0:y1, x0:x1) = 0.95;
end

home = [3, 3];   % [x,y] with 1-based indexing
cur  = home;

% Robot/budget params
E_step = 1.2; E_base = 0.10; T_step = 1.0;
E_reserve = 15.0;
E = 160.0; T = 140.0;

% Sensor params
r = 3;
p_occ_obs = 0.95; p_free_obs = 0.05;

% Initial observation
belief = apply_observation(belief, truth, cur, r, p_occ_obs, p_free_obs);

goals = 0; stepsDone = 0;

while true
    frontiers = detect_frontiers(belief, 0.35);
    if isempty(frontiers)
        disp("No frontiers left."); break;
    end

    [goal, path] = pick_goal_budgeted(belief, frontiers, cur, home, ...
        E, T, E_step, E_base, T_step, E_reserve, r, 0.35);

    if isempty(goal)
        disp("No feasible frontier under budget. Returning."); break;
    end

    % Execute path (skip first cell)
    for i = 2:size(path,1)
        E = E - (E_step + E_base);
        T = T - T_step;
        stepsDone = stepsDone + 1;
        cur = path(i,:);
        belief = apply_observation(belief, truth, cur, r, p_occ_obs, p_free_obs);
        if (E < E_reserve) || (T <= 0)
            disp("Budget exhausted mid-path."); break;
        end
    end

    goals = goals + 1;
    if (E < E_reserve) || (T <= 0), break; end
    if goals >= 40, disp("Stop: demo goal limit."); break; end
end

fprintf("Final pose: (%d,%d) steps=%d goals=%d\n", cur(1), cur(2), stepsDone, goals);
fprintf("Remaining budget: E=%.2f T=%.2f\n", E, T);

% --------------------------- Helper functions ---------------------------

function frontiers = detect_frontiers(belief, free_thr)
    [H,W] = size(belief);
    frontiers = [];
    for y=1:H
        for x=1:W
            if belief(y,x) >= free_thr, continue; end
            nbs = neighbors4([x,y], W, H);
            for k=1:size(nbs,1)
                xn=nbs(k,1); yn=nbs(k,2);
                if abs(belief(yn,xn) - 0.5) < 0.15
                    frontiers = [frontiers; x,y]; %#ok<AGROW>
                    break;
                end
            end
        end
    end
end

function nbs = neighbors4(c, W, H)
    x=c(1); y=c(2);
    cand = [x+1,y; x-1,y; x,y+1; x,y-1];
    keep = cand(:,1)>=1 & cand(:,1)<=W & cand(:,2)>=1 & cand(:,2)<=H;
    nbs = cand(keep,:);
end

function belief = apply_observation(belief, truth, pose, r, p_occ_obs, p_free_obs)
    [H,W] = size(belief);
    x0=pose(1); y0=pose(2);
    for dy=-r:r
        for dx=-r:r
            x=x0+dx; y=y0+dy;
            if x<1 || x>W || y<1 || y>H, continue; end
            isOcc = truth(y,x) > 0.5;
            belief(y,x) = (isOcc)*p_occ_obs + (~isOcc)*p_free_obs;
        end
    end
end

function ig = expected_info_gain(belief, pose, r)
    % nats, approximate entropy reduction proxy
    x0=pose(1); y0=pose(2);
    [H,W] = size(belief);
    ig = 0.0;
    Hpost = 0.5*bern_entropy(0.95) + 0.5*bern_entropy(0.05);
    for dy=-r:r
        for dx=-r:r
            x=x0+dx; y=y0+dy;
            if x<1 || x>W || y<1 || y>H, continue; end
            p0 = belief(y,x);
            H0 = bern_entropy(p0);
            w = max(0.0, min(1.0, 1.0 - abs(p0-0.5)*2.0));
            ig = ig + w * max(0.0, H0 - Hpost);
        end
    end
end

function H = bern_entropy(p)
    p = max(1e-9, min(1-1e-9, p));
    H = -(p*log(p) + (1-p)*log(1-p));
end

function [goal, bestPath] = pick_goal_budgeted(belief, frontiers, cur, home, ...
        E, T, E_step, E_base, T_step, E_reserve, r, lam)

    goal = []; bestPath = [];
    bestU = -1e18;

    % subsample
    if size(frontiers,1) > 80
        idx = randperm(size(frontiers,1), 80);
        frontiers = frontiers(idx,:);
    end

    for k=1:size(frontiers,1)
        g = frontiers(k,:);
        path = astar_grid(belief, cur, g, 0.65);
        if isempty(path), continue; end

        steps = size(path,1) - 1;
        Ego = steps*(E_step + E_base);
        Tgo = steps*T_step;

        back = astar_grid(belief, g, home, 0.65);
        if isempty(back), continue; end
        bsteps = size(back,1) - 1;
        Eback = bsteps*(E_step + E_base);
        Tback = bsteps*T_step;

        if (E - (Ego+Eback) < E_reserve), continue; end
        if (T - (Tgo+Tback) < 0), continue; end

        ig = expected_info_gain(belief, g, r);
        U = ig - lam*steps;

        if U > bestU
            bestU = U;
            goal = g;
            bestPath = path;
        end
    end
end

function path = astar_grid(belief, s, g, occThr)
    % 4-neighbor A*; blocks if belief(y,x) > occThr
    [H,W] = size(belief);
    if any(s<1) || s(1)>W || s(2)>H || any(g<1) || g(1)>W || g(2)>H
        path = []; return;
    end
    if belief(s(2),s(1)) > occThr || belief(g(2),g(1)) > occThr
        path = []; return;
    end

    % maps (use linear indices)
    idx = @(x,y) (y-1)*W + x;
    pos = @(k) [mod(k-1,W)+1, floor((k-1)/W)+1];

    open = java.util.PriorityQueue();
    gscore = containers.Map('KeyType','int32','ValueType','double');
    parent = containers.Map('KeyType','int32','ValueType','int32');

    hs = abs(s(1)-g(1)) + abs(s(2)-g(2));
    open.add({hs, idx(s(1),s(2))});
    gscore(idx(s(1),s(2))) = 0.0;

    visited = containers.Map('KeyType','int32','ValueType','logical');

    while ~open.isEmpty()
        node = open.remove();
        curk = node{2};
        if isKey(visited, curk), continue; end
        visited(curk) = true;

        c = pos(curk);
        if all(c==g)
            % reconstruct
            path = c;
            while isKey(parent, curk)
                curk = parent(curk);
                path = [pos(curk); path]; %#ok<AGROW>
            end
            return;
        end

        nbs = neighbors4(c, W, H);
        for i=1:size(nbs,1)
            nb = nbs(i,:);
            if belief(nb(2),nb(1)) > occThr, continue; end
            nbk = idx(nb(1),nb(2));
            tent = gscore(curk) + 1.0;
            if ~isKey(gscore, nbk) || tent < gscore(nbk)
                gscore(nbk) = tent;
                parent(nbk) = curk;
                f = tent + abs(nb(1)-g(1)) + abs(nb(2)-g(2));
                open.add({f, nbk});
            end
        end
    end
    path = [];
end
