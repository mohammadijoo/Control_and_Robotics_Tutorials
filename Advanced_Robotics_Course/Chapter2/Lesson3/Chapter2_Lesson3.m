function demo_lattice_planner
    % Grid parameters
    DX = 1.0; DY = 1.0; N_THETA = 16;
    NX = 50; NY = 30;
    L = 2.0; V = 1.0; DT = 1.0;

    occ = zeros(NX, NY);  % binary occupancy

    start = [2, 2, angle_index(0.0, N_THETA)];
    goal  = [40, 20, angle_index(0.0, N_THETA)];

    prims = make_primitives(L, V, DT);

    [path, cost] = astar_lattice(start, goal, prims, occ, DX, DY, N_THETA);
    fprintf('Path length = %d, cost = %.3f\n', size(path,1), cost);

    % To connect with Simulink:
    % 1) Export the sequence of continuous poses for each primitive.
    % 2) Use a 'From Workspace' block feeding a trajectory-following controller.

end

function kth = angle_index(theta, N_THETA)
    theta = mod(theta + 2*pi, 2*pi);
    kth = round(theta / (2*pi) * N_THETA);
    kth = mod(kth, N_THETA);
end

function theta = angle_from_index(kth, N_THETA)
    theta = 2*pi * kth / N_THETA;
end

function prims = make_primitives(L, V, DT)
    steer = [-0.4, 0.0, 0.4];
    prims = struct('dx_ref', {}, 'dy_ref', {}, 'dth_ref', {}, 'length', {});
    for i = 1:numel(steer)
        phi = steer(i);
        kappa = tan(phi) / L;
        if abs(kappa) < 1e-6
            dx = V * DT; dy = 0.0; dth = 0.0;
            len = V * DT;
        else
            dth = V * kappa * DT;
            thEnd = dth;
            dx = (1.0 / kappa) * sin(thEnd);
            dy = -(1.0 / kappa) * (cos(thEnd) - 1.0);
            len = abs(dth / kappa);
        end
        prims(i).dx_ref = dx;
        prims(i).dy_ref = dy;
        prims(i).dth_ref = dth;
        prims(i).length = len;
    end
end

function [path, total_cost] = astar_lattice(start, goal, prims, occ, DX, DY, N_THETA)
    NX = size(occ,1); NY = size(occ,2);

    function b = is_free(ix, iy)
        b = (ix >= 1) && (ix <= NX) && (iy >= 1) && (iy <= NY) ...
            && (occ(ix,iy) == 0);
    end

    function h = heuristic(s)
        x = s(1) * DX; y = s(2) * DY;
        xg = goal(1) * DX; yg = goal(2) * DY;
        h = hypot(x - xg, y - yg);
    end

    % open set: columns [ix, iy, kth, g, f]
    open = [start, 0.0, heuristic(start)];
    gmap = containers.Map();
    parent = containers.Map('KeyType','char','ValueType','char');

    key_start = key_of(start);
    gmap(key_start) = 0.0;

    closed = containers.Map('KeyType','char','ValueType','logical');

    while ~isempty(open)
        % pick node with smallest f
        [~, idx] = min(open(:,5));
        curr = open(idx, 1:3);
        gcurr = open(idx, 4);
        open(idx,:) = [];

        kcurr = key_of(curr);
        if isKey(closed, kcurr), continue; end
        closed(kcurr) = true;

        if curr(1) == goal(1) && curr(2) == goal(2)
            % reconstruct path
            path = curr;
            while ~strcmp(kcurr, key_start)
                p = str2num(parent(kcurr)); %#ok<ST2NM>
                path = [p; path]; %#ok<AGROW>
                kcurr = key_of(p);
            end
            total_cost = gcurr;
            return;
        end

        theta = angle_from_index(curr(3), N_THETA);
        for i = 1:numel(prims)
            p = prims(i);
            gx = curr(1) * DX; gy = curr(2) * DY;
            dx = cos(theta)*p.dx_ref - sin(theta)*p.dy_ref;
            dy = sin(theta)*p.dx_ref + cos(theta)*p.dy_ref;
            gx2 = gx + dx; gy2 = gy + dy;
            th2 = theta + p.dth_ref;
            ix2 = round(gx2 / DX);
            iy2 = round(gy2 / DY);
            kth2 = angle_index(th2, N_THETA);
            if ~is_free(ix2, iy2), continue; end
            s2 = [ix2, iy2, kth2];

            key2 = key_of(s2);
            new_g = gcurr + p.length;
            if ~isKey(gmap, key2) || new_g < gmap(key2)
                gmap(key2) = new_g;
                parent(key2) = kcurr;
                f = new_g + heuristic(s2);
                open = [open; s2, new_g, f]; %#ok<AGROW>
            end
        end
    end

    path = [];
    total_cost = inf;
end

function k = key_of(s)
    % string key for map
    k = sprintf('%d,%d,%d', s(1), s(2), s(3));
end
      
