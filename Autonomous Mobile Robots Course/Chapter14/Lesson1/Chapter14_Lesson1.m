% Chapter14_Lesson1.m
% Layered navigation architecture demo in MATLAB/Simulink:
%   - Global layer: plan on an occupancy grid (black-box A* planner)
%   - Local layer: track waypoints with a simple heading controller
%   - Reactive layer: safety filter that scales/halts the command near obstacles
%
% Requires (recommended):
%   - Robotics System Toolbox (for occupancyMap / binaryOccupancyMap)
%   - Navigation Toolbox (for plannerAStarGrid), if available
%   - Simulink (optional; model auto-generation below)
%
% Author: Abolfazl Mohammadijoo (course material)
% License: MIT (see repository)

function Chapter14_Lesson1()
    rng(0);

    % ----------------------------
    % 1) Build a toy occupancy map
    % ----------------------------
    res = 5;             % cells per meter
    mapSize = [12 8];    % meters (x,y)
    occ = binaryOccupancyMap(mapSize(1), mapSize(2), res);

    % Place some random rectangular obstacles
    setOccupancy(occ, [3 2; 3.2 2; 3.2 6; 3 6], 1);
    setOccupancy(occ, [6 0.8; 6.2 0.8; 6.2 4.5; 6 4.5], 1);
    setOccupancy(occ, [8.5 3; 8.7 3; 8.7 7.2; 8.5 7.2], 1);

    start = [1.0 1.0];
    goal  = [11.0 7.0];

    % ------------------------------------
    % 2) Global layer: compute a grid path
    % ------------------------------------
    path = [];
    try
        planner = plannerAStarGrid(occ);
        pathObj = plan(planner, start, goal);
        path = pathObj.States(:,1:2);
    catch
        % Fallback: straight line with sparse waypoints
        path = [linspace(start(1), goal(1), 25)' linspace(start(2), goal(2), 25)'];
        warning('plannerAStarGrid not found; using straight-line fallback path.');
    end

    % ---------------------------------------------
    % 3) Local + reactive layers: waypoint tracking
    % ---------------------------------------------
    pose = [start(1); start(2); 0.0];   % [x;y;theta]
    dt = 0.05;
    vRef  = 0.5;
    kYaw  = 2.0;
    dStop = 0.35;
    dSlow = 0.9;

    traj = zeros(3, 400);
    for k = 1:size(traj,2)
        % choose a lookahead waypoint (simple)
        tgt = pickLookahead(path, pose(1:2)', 0.6);

        % nominal local command
        [vNom, wNom] = localNominal(pose, tgt, vRef, kYaw);

        % reactive safety: use distance-to-nearest-occupied-cell proxy
        d = approxMinObstacleDistance(occ, pose(1:2)');
        [v, w] = safetyFilter(vNom, wNom, d, dStop, dSlow);

        % integrate unicycle kinematics
        pose(1) = pose(1) + v*cos(pose(3))*dt;
        pose(2) = pose(2) + v*sin(pose(3))*dt;
        pose(3) = wrapToPi(pose(3) + w*dt);

        traj(:,k) = pose;

        if norm(pose(1:2)' - goal) < 0.25
            traj = traj(:,1:k);
            break;
        end
    end

    % ----------------------------
    % 4) Plot (optional)
    % ----------------------------
    figure; show(occ); hold on;
    plot(path(:,1), path(:,2), 'LineWidth', 2);
    plot(traj(1,:), traj(2,:), 'LineWidth', 2);
    plot(start(1), start(2), 'o', 'MarkerSize', 8, 'LineWidth', 2);
    plot(goal(1), goal(2), 'x', 'MarkerSize', 10, 'LineWidth', 2);
    legend('map','global path','executed traj','start','goal');
    title('Layered Navigation: Global path + Local tracking + Reactive safety');

    % -----------------------------------------
    % 5) (Optional) Generate a Simulink template
    % -----------------------------------------
    try
        createSimulinkLayeredNavModel();
    catch ME
        warning("Simulink model generation skipped: %s", ME.message);
    end
end

% ---------------- Helper functions ----------------

function p = pickLookahead(path, xy, L)
    if isempty(path), p = xy; return; end
    d2 = sum((path - xy).^2, 2);
    [~, imin] = min(d2);
    acc = 0.0;
    prev = path(imin,:);
    for j = (imin+1):size(path,1)
        cur = path(j,:);
        acc = acc + norm(cur - prev);
        if acc >= L, p = cur; return; end
        prev = cur;
    end
    p = path(end,:);
end

function [v, w] = localNominal(pose, tgt, vRef, kYaw)
    dx = tgt(1) - pose(1);
    dy = tgt(2) - pose(2);
    des = atan2(dy, dx);
    e = wrapToPi(des - pose(3));
    w = kYaw * e;
    v = vRef * max(0.0, cos(e));
end

function [v, w] = safetyFilter(vNom, wNom, d, dStop, dSlow)
    if d <= dStop
        v = 0.0; w = 0.0; return;
    end
    if d <= dSlow
        s = (d - dStop) / (dSlow - dStop + 1e-9);
        s = max(0.0, min(1.0, s));
        v = s * vNom; w = s * wNom; return;
    end
    v = vNom; w = wNom;
end

function d = approxMinObstacleDistance(occ, xy)
    % Approximate min distance by sampling a small neighborhood.
    % This is NOT a full distance transform; used only as a simple proxy.
    r = 1.5;   % meters
    step = 0.1;
    d = inf;
    for dx = -r:step:r
        for dy = -r:step:r
            p = xy + [dx dy];
            if p(1) < 0 || p(2) < 0, continue; end
            if p(1) > occ.XWorldLimits(2) || p(2) > occ.YWorldLimits(2), continue; end
            if getOccupancy(occ, p) > 0.5
                d = min(d, norm([dx dy]));
            end
        end
    end
    if isinf(d), d = 10.0; end
end

function a = wrapToPi(a)
    while a > pi, a = a - 2*pi; end
    while a < -pi, a = a + 2*pi; end
end

function createSimulinkLayeredNavModel()
    % Programmatically create a simple Simulink model template showing the three layers.
    mdl = "Chapter14_Lesson1_LayeredNav";
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl);
    open_system(mdl);

    % Add three subsystems
    add_block('simulink/Ports & Subsystems/Subsystem', mdl + "/GlobalLayer", 'Position',[80 80 220 160]);
    add_block('simulink/Ports & Subsystems/Subsystem', mdl + "/LocalLayer",  'Position',[320 80 460 160]);
    add_block('simulink/Ports & Subsystems/Subsystem', mdl + "/ReactiveLayer", 'Position',[560 80 720 160]);

    % Add Inports/Outports for conceptual wiring
    add_block('simulink/Sources/In1', mdl + "/Map+Goal", 'Position',[20 105 50 125]);
    add_block('simulink/Sources/In1', mdl + "/State+Sensors", 'Position',[20 205 90 225]);
    add_block('simulink/Sinks/Out1', mdl + "/CmdVel", 'Position',[780 105 830 125]);

    % Lines: Map+Goal -> Global -> Local; State+Sensors -> Local and Reactive; Local -> Reactive -> CmdVel
    add_line(mdl, "Map+Goal/1", "GlobalLayer/1");
    add_line(mdl, "GlobalLayer/1", "LocalLayer/1");
    add_line(mdl, "State+Sensors/1", "LocalLayer/2", 'autorouting','on');
    add_line(mdl, "State+Sensors/1", "ReactiveLayer/2", 'autorouting','on');
    add_line(mdl, "LocalLayer/1", "ReactiveLayer/1");
    add_line(mdl, "ReactiveLayer/1", "CmdVel/1");

    % Annotate
    annotation(mdl,'Text','String','Layered navigation template: Global -> Local -> Reactive safety','Position',[120 10 700 40]);

    save_system(mdl);
end
