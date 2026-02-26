function facts_out = call_grasp_stream(block, facts_in)
    if ~ismember({"movable", block}, facts_in)
        facts_out = {};
        return;
    end
    theta = 2*pi*rand();
    g = {"top_grasp", theta};
    facts_out = {{"grasp", block, g}};
end

function facts_out = call_motion_stream(q_start, q_goal, robot, env)
    % robot: rigidBodyTree, env: collision world
    ss = stateSpaceRigidBodyTree(robot);
    sv = validatorRigidBodyTree(ss, "Environment", env);
    planner = plannerRRTStar(ss, sv);
    planner.MaxConnectionDistance = 0.1;
    planner.MaxIterations = 300;
    [pthObj, solInfo] = plan(planner, q_start, q_goal);
    if solInfo.IsPathFound
        facts_out = {{"reachable", q_start, q_goal}};
    else
        facts_out = {};
    end
end

% Focused loop (symbolic planner not implemented here)
knownFacts = {{"movable", "b1"}};
for it = 1:10
    % symbolic planner stub
    planSkeleton = {"pick b1", "place b1"};
    % stream calls
    facts_new = call_grasp_stream("b1", knownFacts);
    if isempty(facts_new)
        knownFacts{end+1} = {"failed-stream", "grasp-b1"};
        continue;
    end
    knownFacts = [knownFacts, facts_new]; %#ok<AGROW>
    % Suppose we have robot, env, q_start, q_goal defined in workspace
    facts_new = call_motion_stream(q_start, q_goal, robot, env);
    if isempty(facts_new)
        knownFacts{end+1} = {"failed-stream", "motion-b1"};
    else
        knownFacts = [knownFacts, facts_new]; %#ok<AGROW>
        disp("Realized TAMP plan.");
        break;
    end
end
      
