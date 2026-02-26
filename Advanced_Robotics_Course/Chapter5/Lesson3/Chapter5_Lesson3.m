function [tree, bestGoal] = kinodynamic_rrt_star_matlab()
    % State: [x; y; vx; vy]
    xInit = [1; 1; 0; 0];
    bounds = [0 10 0 10 2]; % [xmin xmax ymin ymax vmax]
    uSet = [ -1 0; 1 0; 0 -1; 0 1; 0 0 ];
    dt = 0.1; steps = 5;
    maxIter = 500; gamma = 30.0; dim = 4;

    node.state = xInit;
    node.parent = 0;
    node.cost = 0;
    tree = node;
    bestGoal = [];

    for n = 1:maxIter
        xRand = [ ...
            bounds(1) + (bounds(2) - bounds(1)) * rand; ...
            bounds(3) + (bounds(4) - bounds(3)) * rand; ...
            -bounds(5) + 2 * bounds(5) * rand; ...
            -bounds(5) + 2 * bounds(5) * rand ];

        % nearest
        dMin = inf; idxNear = 1;
        for i = 1:numel(tree)
            d = state_distance(tree(i).state, xRand);
            if d < dMin
                dMin = d; idxNear = i;
            end
        end

        xNear = tree(idxNear).state;
        bestEdgeCost = inf;
        bestXnew = [];

        for ui = 1:size(uSet,1)
            u = uSet(ui,:).';
            % forward simulate via Simulink
            [xNew, edgeCost] = propagate_with_simulink( ...
                xNear, u, dt, steps);
            if isempty(xNew), continue; end

            if xNew(1) < bounds(1) || xNew(1) > bounds(2) || ...
               xNew(2) < bounds(3) || xNew(2) > bounds(4)
                continue;
            end

            if edgeCost < bestEdgeCost
                bestEdgeCost = edgeCost;
                bestXnew = xNew;
            end
        end

        if isempty(bestXnew), continue; end

        rn = (gamma * (log(n) / n))^(1/dim); %#ok<NASGU>

        newNode.state = bestXnew;
        newNode.parent = idxNear;
        newNode.cost = tree(idxNear).cost + bestEdgeCost;
        tree(end+1) = newNode; %#ok<AGROW>

        if is_in_goal(bestXnew)
            if isempty(bestGoal) || newNode.cost < bestGoal.cost
                bestGoal = newNode;
            end
        end
    end
end

function d = state_distance(x1, x2)
    wPos = 1.0; wVel = 0.1;
    dp = x1(1:2) - x2(1:2);
    dv = x1(3:4) - x2(3:4);
    d = sqrt(wPos * (dp.'*dp) + wVel * (dv.'*dv));
end

function [xNew, edgeCost] = propagate_with_simulink(xInit, u, dt, steps)
    Tfinal = dt * steps;
    % Assume Simulink model 'double_integrator_2d' with input u and initial state xInit
    simOut = sim("double_integrator_2d", ...
                 "StopTime", num2str(Tfinal), ...
                 "LoadInitialState", "on", ...
                 "InitialState", xInit, ...
                 "SrcWorkspace", "current");
    xTraj = simOut.xout.signals.values;
    xNew = xTraj(end,:).';
    edgeCost = Tfinal; % time cost
end

function flag = is_in_goal(x)
    goalCenter = [9; 9];
    goalR = 0.5;
    flag = norm(x(1:2) - goalCenter) <= goalR;
end
      
