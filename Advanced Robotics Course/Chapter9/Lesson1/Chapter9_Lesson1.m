function tamp_demo()
    % Symbolic states encoded as integer ids for simplicity
    % 1: HandEmpty & On(Box,Table)
    % 2: Holding(Box)
    % 3: HandEmpty & On(Box,Shelf)
    INIT = 1;
    GOAL = 3;

    % Actions: [preState nextState id]
    actions = [...
        1 2 1;  % PickFromTable
        2 3 2   % PlaceOnShelf
    ];

    plan = breadthFirstTAMP(INIT, GOAL, actions);
    disp('Symbolic action ids:'), disp(plan);
end

function plan = breadthFirstTAMP(initState, goalState, actions)
    queue = {struct('state', initState, 'plan', [])};
    visited = initState;
    head = 1;
    plan = [];
    while head <= numel(queue)
        node = queue{head}; head = head + 1;
        if node.state == goalState
            plan = node.plan;
            return;
        end
        for i = 1:size(actions,1)
            a = actions(i,:);
            pre = a(1); nxt = a(2); aid = a(3);
            if pre ~= node.state
                continue;
            end
            if ismember(nxt, visited)
                continue;
            end
            if ~checkGeomTransition(node.state, nxt, aid)
                continue;
            end
            visited(end+1) = nxt; %#ok<AGROW>
            child.state = nxt;
            child.plan  = [node.plan, aid];
            queue{end+1} = child; %#ok<AGROW>
        end
    end
end

function ok = checkGeomTransition(s, sNext, aid)
    % In a full implementation, this would:
    % 1) Map symbolic states s, sNext to robot configurations using a
    %    refinement map.
    % 2) Invoke a motion planner (e.g. manipulatorRRT) to check whether
    %    a collision-free path exists.
    % Here we just return true as a placeholder.
    ok = true;
end
      
